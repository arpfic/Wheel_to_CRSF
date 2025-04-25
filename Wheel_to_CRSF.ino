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
#include "crsf_protocol.h"
#include "CrsfSerial.h"

/* ----------------- Broches & constantes ------------------------- */
constexpr uint8_t  PIN_RX = 15;          // entrée
constexpr uint8_t  PIN_TX = 4;           // sortie via 1 kΩ
constexpr uint32_t CRSF_BAUD   = 420000; // 250 Hz
constexpr uint32_t PERIOD_US   = 4000;   // 4 ms

/* ----------------- Objets -------------------------------------- */
HardwareSerial uart(1);
CrsfSerial     crsf(uart, CRSF_BAUD);

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
}

/* ----------------- LOOP ---------------------------------------- */
void loop()
{
    static uint32_t tLast = 0;
    crsf.loop();                                  // lit la télémétrie

    if (micros() - tLast >= PERIOD_US) {
        static bool left = false;
        static uint32_t tToggle = 0;
        if (millis() - tToggle > 10000) { left = !left; tToggle = millis(); }

        for (uint8_t ch=1; ch<=CRSF_NUM_CHANNELS; ++ch) crsf.setChannel(ch,1500);
        crsf.setChannel(2, left ? 988 : 2012);    // direction test
        crsf.queuePacketChannels();

        tLast = micros();
    }
}
