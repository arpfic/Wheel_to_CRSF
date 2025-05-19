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

//-> Refaire l'arming : inverser la logique, mettre un SPDT, et voir si on met les infos du volant
//-> PCB
//-> support d'écran
//-> paddles du volant : passage de vitesses ?
//-> runcam : configurer l'esd
//-> usiner une piece pour les amortisseurs
 
#include <Arduino.h>
// Elrs
#include "crsf_protocol.h"
#include "CrsfSerial.h"
// Volant
#include "HID_Wheel.h"
#include <usbhub.h>

// Ecran
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define STR_TRIM 14
#define SPD_MAX 27


#define ARMING_BUTTON 34

#define CRSF_CHAN_MIN 1000
#define CRSF_CHAN_MAX 2000
#define CRSF_CHAN_MID 1500

#define CRSF_VITESSE_MIN 1320

#define OFFSET_VOLANT 0
#define OFFSET_CAR 0
#define OFFSET_CAR_1_18 158

// l'ESC dit, en dessous de 3.5/cell on fait un cutoff
#define BATTERY_LOW_CUT 7.2
#define BAT_MAX_MS 20000

/* ----------------- Broches & constantes ------------------------- */
constexpr uint8_t  PIN_RX = 2;           // entrée
constexpr uint8_t  PIN_TX = 4;           // sortie via 1 kΩ
constexpr uint32_t CRSF_BAUD         = 420000; // 250 Hz
constexpr uint32_t PERIOD_RC_US      = 4000;   // 4 ms
constexpr uint32_t PERIOD_FORCE_MS   = 100;   // 150ms
constexpr uint32_t PERIOD_SCREEN_MS  = 100;   // 150ms
constexpr uint32_t PERIOD_POTS_MS    = 100;   // 150ms

/* ----------------- Objets -------------------------------------- */
HardwareSerial uart(1);
CrsfSerial     Crsf(uart, CRSF_BAUD);

// Shield USB et Volant
USB          Usb;
USBHub       Hub(&Usb);
HID_Wheel    wheel(&Usb);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int offset_volant_value = 0;
int offset_vitesse_value = 0;

static uint8_t launched = 0;
static uint8_t bouing = 0;
static uint8_t armed = 0;

char flight_mode[9];
char link_info[32]     = "";             // "LQ et RSSI"
char battery_info[32]  = "";             // "TENSION/% 55 %"
char linkStr[16]       = "LINK ???";     // affichage
bool link_status       = false;          // état logiciel

float battery_voltage  = 8.0;            // Retour de tension de batterie
uint32_t vmaxStamp     = 0;

uint32_t last_telemetry = 0; 

/* ----------------- Paquets pour le volant ---------------------- */
// Angle configuré
uint8_t packetAngle[64] = {
    0x60, 0x08, 0x11, 0x5e, 0x42
};

// Une communication USB au volant comment par packetInit et termine
// par packetEnd
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

// Envoie un paquet de force de volant
void sendForce(uint8_t pct)
{
    if (pct > 100) pct = 100;

    uint8_t buf64[64] = {0};          // tampon HID plein
    buf64[0] = 0x60;
    buf64[1] = 0x02;
    buf64[2] = (pct * 255) / 100;     // 0–100 % → 0x00–0xFF

    Usb.outTransfer(wheel.GetAddress(),
                    ENDPOINT_THRUST_NEW,
                    64, buf64);
}

/* ----------------- Handshake PING -> INFO ----------------------- */
// Pour le hook, mais jamais utilisée en pratique
// https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md#0x29-parameter-device-information
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
            Crsf.queuePacket(CRSF_ADDRESS_CRSF_TRANSMITTER,
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
    snprintf(link_info, sizeof(link_info),
             "LQ:%u RSSI:%d",            // ex. "LQ:97 RSSI:-46"
             l->uplink_Link_quality,
             l->uplink_RSSI_1);

    Serial.printf("[LINK] %s SNR:%d\n",
                  link_info, l->uplink_SNR);

    // On met à jour la detection de lien perdu
    last_telemetry = millis();
}

void onBatt(crsf_sensor_battery_t *b)
{
    float b_vol = be16toh(b->voltage) / 10.0f;   // dixièmes de volt → volt
    uint8_t battery_percent = b->remaining;

    /* ---------------- reinit toutes les 30s ------------------- */
    if (millis() - vmaxStamp > BAT_MAX_MS) {          // 30 000 ms
        battery_voltage = b_vol;
        vmaxStamp       = millis();
    }

    /* ---------- mise à jour du pic courant -------------------- */
    if (b_vol > battery_voltage) {
        battery_voltage = b_vol;
        vmaxStamp       = millis();
    }

    snprintf(battery_info, sizeof(battery_info), 
             "%.1fA %.1fV %u%%",                // ex. "15.3 V 67 %"
             be16toh(b->current) / 10.0f, b_vol, battery_percent);

    Serial.printf("[BATT] %s\n",
                  battery_info);

    Serial.println(battery_voltage);
}


void onGps (crsf_sensor_gps_t *g)
{
    Serial.printf("[GPS]  %.7f  %.7f  alt:%d  sats:%u\n",
                  g->latitude /1e7, g->longitude/1e7,
                  g->altitude, g->satellites);
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
    while (n && mode[n-1] == ' ') mode[--n] = '\0';

    bool isAcro = (strcasestr(mode, "ACRO") != nullptr);
    strcpy(flight_mode, isAcro ? "ARMED" : "DESARMED");

    Serial.printf("[MODE] %-15s\n", mode);
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

// Not used
void sendAutoArming(){
    Serial.println("Arming. please wait");
    sendArmingPacket(1000, 5000);
    Serial.println("Arming. Sending 2000");
    sendArmingPacket(2000, 1000);
    Serial.println("Arming. Sending 1000");
    sendArmingPacket(1000, 1000);
    Serial.println("Arming. Sending 2000");
    sendArmingPacket(2000, 1000);
    Serial.println("Arming. Sending 1000");
    sendArmingPacket(1000, 1000);    
}

/* ----------------- Arming sur le canal 5 ------------------------ */
// Not used
void sendArmingPacket(uint16_t value, uint32_t duree_ms)
{
    const uint32_t start = millis();
    while (millis() - start < duree_ms)
    {
        /* Prépare les 16 canaux à MID*/
        for (uint8_t ch = 1; ch <= CRSF_NUM_CHANNELS; ++ch)
            Crsf.setChannel(ch, CRSF_CHAN_MID);
        Crsf.setChannel(5, value);   // CH5 à la valeur voulue
        /* Envoie le paquet */
        Crsf.queuePacketChannels();
        /* Cadence = 250 Hz */
        delayMicroseconds(PERIOD_RC_US);
    }
}

void debugShowFPS()
{
    static uint32_t lastStamp = 0;
    static uint32_t counter   = 0;

    ++counter;

    uint32_t now = millis();
    if (now - lastStamp >= 1000) {
        Serial.printf("[FPS] %luHz\n", counter);
        counter   = 0;
        lastStamp = now;
    }
}

/* ----------------- SETUP --------------------------------------- */
void setup()
{
    Serial.begin(115200);
    pinMode(PIN_RX, INPUT_PULLUP);
    // PUSHTEST
    pinMode(ARMING_BUTTON, INPUT);

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    delay(500);
    display.clearDisplay();

    uart.begin(CRSF_BAUD, SERIAL_8N1, PIN_RX, PIN_TX, true); // inversé

    Crsf.begin();
    Crsf.onOobData              = onOob;
    Crsf.onPacketLinkStatistics = onLink;
    Crsf.onPacketGps            = onGps;
    Crsf.onPacketBattery        = onBatt;
    Crsf.onPacketEsc            = onEsc;
    Crsf.onPacketFlightMode     = onFlight;

    // Initialiser le shield USB
    if (Usb.Init() == -1) {
      Serial.println("USB Shield did not start.");
    }

    /* Rafale neutre 1000 ms pour sortir le RX du failsafe */
    const uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
        for (uint8_t ch=1; ch<=CRSF_NUM_CHANNELS; ++ch) Crsf.setChannel(ch, CRSF_CHAN_MID);
        Crsf.queuePacketChannels();
        delayMicroseconds(PERIOD_RC_US);
    }
}

/* ----------------- LOOP ---------------------------------------- */
void loop()
{
    static uint32_t refreshRC      = 0;
    static uint32_t refreshForce   = 0;
    static uint32_t refreshScreen  = 0;
    static uint32_t refreshPots    = 0;

    // Gère la pile USB, déclenche Parse() quand un rapport arrive
    Usb.Task();
    // Gère la pile CRSF
    Crsf.loop();

    if (wheel.connected()) {
        // Configure le retour de force et l'angle
        if (launched == 0){
            sendPacket(packetInit);
            sendForce(0);
            sendPacket(packetAngle);
            sendPacket(packetEnd);
            launched = 1;
        }

//      if (wheel.bouing_value == 1){
//        sendBouing();
//      }
//      if (armed == 0){
//          sendAutoArming();
//          armed = 1;
//      }

        // 150 Hz
        if (millis() - refreshForce >= PERIOD_FORCE_MS){

            uint16_t vitesse_us = constrain(CRSF_CHAN_MID
                                            + wheel.accel_value
                                            - wheel.frein_value,
                                            CRSF_VITESSE_MIN, CRSF_CHAN_MAX);

            uint8_t pct;
            // on rend proportionnel la force entre 0 et 100
            if (vitesse_us <= CRSF_CHAN_MID) pct = 0;
            else pct = ( (vitesse_us - CRSF_CHAN_MID) * 100 ) / (CRSF_CHAN_MAX - CRSF_CHAN_MID);   // (2000-1500)=500
            sendForce(pct);
            refreshForce = millis();
        }

        // 250Hz
        if (micros() - refreshRC >= PERIOD_RC_US) {

            // ARMING
            uint16_t arming = 2000;
            // Check Arming AND battery voltage ? add && battery_voltage > BATTERY_LOW_CUT
            if (digitalRead(ARMING_BUTTON) == HIGH) arming = 1000;
            uint16_t volant_us  = constrain(wheel.volant_value  + offset_volant_value, CRSF_CHAN_MIN, CRSF_CHAN_MAX);
            uint16_t vitesse_us = constrain(CRSF_CHAN_MID + (int16_t)(((float)wheel.accel_value * (float)offset_vitesse_value) / 100.0) - wheel.frein_value, CRSF_VITESSE_MIN, CRSF_CHAN_MAX);

            /* remplit les 16 voies */
            for (uint8_t ch = 1; ch <= CRSF_NUM_CHANNELS; ++ch) Crsf.setChannel(ch, CRSF_CHAN_MID);
            Crsf.setChannel(3, vitesse_us);    // CH3 = throttle (ex.)
            Crsf.setChannel(4, volant_us);     // CH4 = steering
            Crsf.setChannel(5, arming);        // CH5 = ARMING_BUTTON status

            Crsf.queuePacketChannels();

            refreshRC = micros();
        }
    }

    // ECRAN
    // 10Hz
    if (millis() - refreshScreen >= PERIOD_SCREEN_MS){
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 10);
        display.printf("OFF. VOLANT: %i\n", offset_volant_value);
        display.setCursor(0, 20);
        display.printf("OFF. SPEED: %i%\n", offset_vitesse_value);
        display.setCursor(0, 30);
        if (millis() - last_telemetry > 1500) {
            display.printf("BAT: ???\n");
            display.setCursor(0, 40);
            display.printf("LINK LOST");
            display.setCursor(0, 50);
            display.printf("STATUS: ???\n");
        } else {
            if (battery_voltage > BATTERY_LOW_CUT) {
                display.printf("BAT: %s\n", battery_info);
                display.setCursor(0, 40);
            } else {
                display.printf("BAT: CHANGE BATTERY!!\n");
                display.setCursor(0, 40);                
            }
            display.printf("%s\n", link_info);
            display.setCursor(0, 50);
            display.printf("STATUS: %s\n", flight_mode);
        }
        display.display();

        refreshScreen = millis();
    }

    // POTS
    // 10Hz
    if (millis() - refreshPots >= PERIOD_POTS_MS){
        offset_volant_value = analogRead(STR_TRIM);
        offset_vitesse_value = analogRead(SPD_MAX);
        
        // Pot d'ajustement du point milieu
        offset_volant_value = map(offset_volant_value,0,4095,-70,70);

        // Pot de réduction de vitesse max : entre 16% (sinon la voiture ne démarre pas)
        // et 100% de la valeur
        offset_vitesse_value = map(offset_vitesse_value,0,4095,16,100);

        refreshPots = millis();
    }

    debugShowFPS();

}
