/********************************************************************
 *  ESP32-WROOM-32 ↔ Module ELRS (1-fil inversé, half-duplex)
 *  TX : GPIO4   (via 1 kΩ série)
 *  RX : GPIO15  (même fil, pull-up au boot)
 *  16 voies RC toutes les 4 ms – CH2 alterne g./d. pour test
 *  Télémétrie : Link, GPS, Battery, ESC
 ********************************************************************/

/*
 *  Fonctionnement :
 *      • 420 kbauds, niveau inversé (convention CRSF).
 *      • Toutes les 4 ms : envoi d’un paquet RC 16 voies (250 Hz).
 *      • CH2 alterne gauche/droite pour vérifier visuellement.
 *      • Le module renvoie LinkStats, GPS, Battery, ESC…
 *      • On répond au DEVICE_PING (0x28) par DEVICE_INFO (0x29).
 */
 
#include <Arduino.h>
// Elrs
#include "crsf_protocol.h"
#include "CrsfSerial.h"
// Volant
#include "HID_Wheel.h"
#include <usbhub.h>

/* ----------------- Broches & constantes ------------------------- */
constexpr uint8_t  PIN_RX = 2;           // entrée
constexpr uint8_t  PIN_TX = 4;           // sortie via 1 kΩ
constexpr uint32_t CRSF_BAUD   = 420000; // 250 Hz
constexpr uint32_t PERIOD_US   = 4000;   // 4 ms

#define CRSF_CHAN_MIN 1000
#define CRSF_CHAN_MAX 2000
#define CRSF_CHAN_MID 1500

#define OFFSET_VOLANT 0
#define OFFSET_CAR 0
#define OFFSET_CAR_1_18 158

/* ----------------- Objets -------------------------------------- */
HardwareSerial uart(1);
CrsfSerial     crsf(uart, CRSF_BAUD);

// Shield USB et Volant
USB          Usb;
USBHub       Hub(&Usb);
HID_Wheel    wheel(&Usb);

static unsigned long lastSend  = 0;
static unsigned long lastPrint = 0;
static uint8_t launched = 0;
static uint8_t bouing = 0;
static uint8_t armed = 0;

/* ----------------- Paquets pour le volant ---------------------- */
uint8_t packetForce0[64] = {
    0x60, 0x02
};

uint8_t packetForce100[64] = {
    0x60, 0x02, 0xff
};

uint8_t packetForce50[64] = {
    0x60, 0x02, 0x7d
};

// Angle
uint8_t packetAngle[64] = {
    0x60, 0x08, 0x11, 0x5e, 0x42
};

uint8_t packetInit[64] = {
    0x60, 0x01, 0x05
};

uint8_t packetEnd[64] = {
    0x60, 0x01
};

uint8_t packet1[64] = {
    0x60, 0x00, 0x01, 0xeb, 0x7a, 0x40, 0xfe, 0xff, 0x00, 0x00, 0x21, 0x00, 0x00, 0x80, 0x95, 0x00, 0xfc, 0x7f, 0xe5, 0x01, 0x03, 0x00, 0x03, 0x4f, 0xbc, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t packet2[64] = {
    0x60, 0x00, 0x01, 0x89, 0x01
};

/* ----------------- Handshake PING → INFO ----------------------- */
// TODO : Vérifier si on a l'équivalent dans Alfredo
void onOob(uint8_t b)               // appelé pour chaque octet « hors CRSF »
{
    static uint8_t buf[6];          // 6 octets = entête complet d’un ping
    static uint8_t pos = 0;
    buf[pos++] = b;

    if (pos >= 2 && pos >= buf[1] + 2) {
        if (buf[0]==CRSF_ADDRESS_CRSF_TRANSMITTER &&
            buf[2]==CRSF_FRAMETYPE_DEVICE_PING)
        {
            // réponse : 10 octets DEVICE_INFO minimalistes
            const uint8_t devInfo[10] =
                {0x12,0x34,0x56,0x78,  // n° série arbitraire
                 0x01,                 // HW ver.
                 0x00,0x01,            // SW v0.1
                 0x00,0x00,0x00};      // reserved + flags
            crsf.queuePacket(CRSF_ADDRESS_CRSF_TRANSMITTER,
                             CRSF_FRAMETYPE_DEVICE_INFO,
                             devInfo, sizeof(devInfo));
        }
        pos = 0;                      // on réarme le mini-parser
    }
    if (pos >= sizeof(buf)) pos = 0;  // sécurité débordement
}

// Arming sur le canal 5
void sendArmCH5(uint16_t value, uint32_t duree_ms)
{
    const uint32_t start = millis();
    while (millis() - start < duree_ms)
    {
        /* Prépare les 16 canaux à MID*/
        for (uint8_t ch = 1; ch <= CRSF_NUM_CHANNELS; ++ch)
            crsf.setChannel(ch, CRSF_CHAN_MID);
        crsf.setChannel(5, value);   // CH5 à la valeur voulue
        /* Envoie le paquet */
        crsf.queuePacketChannels();
        /* Respecte la cadence (250 Hz) */
        delayMicroseconds(PERIOD_US);   // PERIOD_US = 4000 µs
    }
}

/* ----------------- Callbacks telemetry ------------------------ */
void onLink(crsfLinkStatistics_t *l)
{
    Serial.printf("[LINK] LQ:%u RSSI:%d SNR:%d\n",
                  l->uplink_Link_quality,
                  l->uplink_RSSI_1,
                  l->uplink_SNR);
}

void onGps (crsf_sensor_gps_t *g)
{
    Serial.printf("[GPS]  %.7f  %.7f  alt:%d  sats:%u\n",
                  g->latitude /1e7, g->longitude/1e7,
                  g->altitude, g->satellites);
}

void onBatt(crsf_sensor_battery_t *b)
{
    Serial.printf("[BATT] %.1f V  %.1f A  %u %%\n",
                  be16toh(b->voltage)/10.0,
                  be16toh(b->current)/10.0,
                  b->remaining);
}

void onEsc (crsf_sensor_esc_t *e)
{
    float rpm = be32toh(e->erpm) * 0.6f;      // eRPM/100 → RPM
    Serial.printf("[ESC] %.0f RPM  %d °C\n", rpm, e->temperature);
}

void onFlight(crsf_flight_mode_t *fm)
{
    char mode[16];
    memcpy(mode, fm->mode, 15);
    mode[15] = '\0';
    size_t n = strnlen(mode, 15);
    while (n && mode[n-1] == ' ') mode[--n] = '\0';   // trim trailing blanks
    Serial.printf("[MODE] %s\n", mode);
}

// Send to wheel
uint8_t sendPacket(uint8_t *packet) {
    if (!wheel.GetAddress()) {
        Serial.println(F("Périphérique non connecté !"));
        return 0xFF;
    }

    uint8_t rcode = Usb.outTransfer(wheel.GetAddress(), ENDPOINT_THRUST_NEW, 64, packet);
    if (rcode) {
        Serial.print(F("Échec outTransfer, code=0x"));
        Serial.println(rcode, HEX);
    }
    return rcode; // 0 si OK
}

void sendBouing() {
    sendPacket(packetInit);
    delay(10);
    sendPacket(packet1);
    delay(10);
    sendPacket(packet2);
    delay(500);
    sendPacket(packetEnd);
}

/* ----------------- SETUP --------------------------------------- */
void setup()
{
    Serial.begin(115200);
    pinMode(PIN_RX, INPUT_PULLUP);
    uart.begin(CRSF_BAUD, SERIAL_8N1, PIN_RX, PIN_TX, true); // inversé

    crsf.begin();
    crsf.onOobData              = onOob;
    crsf.onPacketLinkStatistics = onLink;
    crsf.onPacketGps            = onGps;
    crsf.onPacketBattery        = onBatt;
    crsf.onPacketEsc            = onEsc;
    crsf.onPacketFlightMode     = onFlight;

    // Initialiser le shield USB
    if (Usb.Init() == -1) {
      Serial.println("USB Shield did not start.");
    }

    /* Rafale neutre 1000 ms pour sortir le RX du failsafe */
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
        for (uint8_t ch=1; ch<=CRSF_NUM_CHANNELS; ++ch) crsf.setChannel(ch, CRSF_CHAN_MID);
        crsf.queuePacketChannels();
        delayMicroseconds(PERIOD_US);
    }
}

/* ----------------- LOOP ---------------------------------------- */
void loop()
{

    // Gère la pile USB, déclenche Parse() quand un rapport arrive
    Usb.Task();

    // gestion du crsf
    static uint32_t tLast = 0;
    crsf.loop();

    if (wheel.connected()) {
      // Configure le retour de force
      if (launched == 0){
        sendPacket(packetInit);
        delay(10);
        sendPacket(packetForce50);
        delay(10);
        sendPacket(packetAngle);
        delay(10);
        sendPacket(packetEnd);
        launched = 1;
      }

//      if (wheel.bouing_value == 1){
//        sendBouing();
//      }

      // Arming
      if (armed == 0){
        Serial.println("Arming. please wait");
        sendArmCH5(1000, 5000);
        Serial.println("Arming. Sending 2000");
        sendArmCH5(2000, 1000);
        Serial.println("Arming. Sending 1000");
        sendArmCH5(1000, 1000);
        Serial.println("Arming. Sending 2000");
        sendArmCH5(2000, 1000);
        Serial.println("Arming. Sending 1000");
        sendArmCH5(1000, 1000);
        armed = 1;
      }

      // 250Hz
      if (micros() - tLast >= PERIOD_US) {
  
      uint16_t volant_us  = constrain(wheel.volant_value  + OFFSET_VOLANT,
                                      CRSF_CHAN_MIN, CRSF_CHAN_MAX);
    
      uint16_t vitesse_us = constrain(CRSF_CHAN_MID + wheel.accel_value - wheel.frein_value,
                                      CRSF_CHAN_MIN, CRSF_CHAN_MAX);
    
      /* remplit les 16 voies */
      for (uint8_t ch = 1; ch <= CRSF_NUM_CHANNELS; ++ch) crsf.setChannel(ch, CRSF_CHAN_MID);
      crsf.setChannel(3, vitesse_us);    // CH3 = throttle (ex.)
      crsf.setChannel(4, volant_us);     // CH4 = steering
      crsf.setChannel(5, 1000);          // CH5 = stay armed

      crsf.queuePacketChannels();

      tLast = micros();

    
      //Serial.print("  volant_value=");
      //Serial.print(volant_us);
      //Serial.print("  vitesse_value=");
      //Serial.println(vitesse_us);
      }
    }
}
