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
constexpr uint8_t  PIN_RX = 15;          // entrée
constexpr uint8_t  PIN_TX = 4;           // sortie via 1 kΩ
constexpr uint32_t CRSF_BAUD   = 420000; // 250 Hz
constexpr uint32_t PERIOD_US   = 4000;   // 4 ms

#define CRSF_CHAN_MIN 172
#define CRSF_CHAN_MAX 1792

#define CRSF_VOLANT_MIN 200
#define CRSF_VOLANT_MAX 1500

#define CRSF_VITESSE_MIN 700
#define CRSF_VITESSE_MAX 1200

#define OFFSET_VOLANT 73
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
// Vérifier si on a l'équivalent dans Alfredo
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

    /* Rafale neutre 200 ms pour sortir le RX du failsafe */
    const uint32_t t0 = millis();
    while (millis() - t0 < 200) {
        for (uint8_t ch=1; ch<=CRSF_NUM_CHANNELS; ++ch) crsf.setChannel(ch,1500);
        crsf.queuePacketChannels();
        delayMicroseconds(PERIOD_US);
    }

    // Initialiser le shield USB
    if (Usb.Init() == -1) {
      Serial.println("USB Shield did not start.");
    }

}

/* ----------------- LOOP ---------------------------------------- */
void loop()
{

    // Gère la pile USB, déclenche Parse() quand un rapport arrive
    Usb.Task();

    // gestion du crsf
    static uint32_t tLast = 0;
    crsf.loop();                                  // lit la télémétrie

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

    if (micros() - tLast >= PERIOD_US) {

      if (wheel.bouing_value == 1){
        sendBouing();
      }

      uint16_t volant_value = constrain(wheel.volant_value + OFFSET_CAR + OFFSET_VOLANT, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
      uint16_t vitesse_value = constrain(CRSF_CHANNEL_VALUE_MID + wheel.accel_value - wheel.frein_value, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);

      // Rempli le paquet
      for (uint8_t ch=2; ch<=CRSF_NUM_CHANNELS; ++ch) crsf.setChannel(ch, CRSF_CHANNEL_VALUE_MID);
      crsf.setChannel(0, vitesse_value);
      crsf.setChannel(1, volant_value);
      crsf.queuePacketChannels();

      tLast = micros();
    }
  }
}
