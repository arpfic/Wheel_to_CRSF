/********************************************************************
 *  ESP32  ➜  Module ELRS (CRSF)
 *  - TX GPIO4, RX GPIO2 (inversés, half‑duplex)
 *  - Envoie 16 voies à 1500 µs toutes les 4 ms
 *  - Affiche tout paquet downlink reçu
 ********************************************************************/
#include <Arduino.h>
#include <HardwareSerial.h>
#include <crc8.h>
#include "CrsfSerial.h"

/* ----------- Broches & constantes --------------------------------*/
#define PIN_RX_OUT         15
#define PIN_TX_OUT         4
constexpr uint32_t CRSF_BAUD        = 420000;   // 250 Hz ELRS
constexpr uint32_t SEND_INTERVAL_US = 4000;     // 4 ms → 250 Hz

/* ----------- Objets globaux --------------------------------------*/
HardwareSerial   CRSFPort(1);                   // UART1
CrsfSerial       crsf(CRSFPort, CRSF_BAUD);
Crc8             crc8(0xD5);                    // générateur CRC CRSF

/* =================================================================
 *                 Utilitaires – construction de trames
 * =================================================================*/
void sendNeutralChannels()
{
    /* ---- 1. Payload : 22 octets, 16 voies x 11 bits ------------ */
    constexpr uint16_t usMid   = 1500;
    constexpr uint16_t crsfMid = US_to_CRSF(usMid);  // ≈ 992

    uint8_t payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE];   // 22 o
    uint8_t *p = payload;
    uint32_t scratch = 0;  uint8_t bits = 0;

    for (uint8_t ch = 0; ch < CRSF_NUM_CHANNELS; ++ch) {
        scratch |= uint32_t(crsfMid) << bits;
        bits += CRSF_BITS_PER_CHANNEL;            // +11 bits
        while (bits >= 8) {                       // vide par octet
            *p++ = scratch & 0xFF;
            scratch >>= 8;
            bits -= 8;
        }
    }

    /* ---- 2. En‑tête complet ------------------------------------ */
    uint8_t frame[1 + 1 + 1 + sizeof(payload) + 1];         // 26 o
    frame[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;               // 0xEE
    frame[1] = sizeof(payload) + 2;                         // len+crc
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;           // 0x16
    memcpy(&frame[3], payload, sizeof(payload));
    frame[sizeof(frame) - 1] = crc8.calc(&frame[2],
                                         sizeof(payload) + 1);

    /* ---- 3. Send ------------------------------------------------ */
    CRSFPort.write(frame, sizeof(frame));
}

/* =================================================================
 *                 Gestion du DEVICE_PING  ->  DEVICE_INFO
 * =================================================================*/
#pragma pack(push,1)
struct crsf_device_info_t {
    uint8_t serial[4]    {0x12, 0x34, 0x56, 0x78};
    uint8_t hw_version   {1};
    uint8_t sw_major     {0};
    uint8_t sw_minor     {1};
    uint8_t reserved1    {0};
    uint8_t device_opts  {0};
    uint8_t empty        {0};
};
#pragma pack(pop)

void sendDeviceInfo()
{
    crsf_device_info_t info;
    crsf.queuePacket(CRSF_ADDRESS_CRSF_TRANSMITTER,
                     CRSF_FRAMETYPE_DEVICE_INFO,
                     &info, sizeof(info));
}

void onOobByte(uint8_t b)
{
    static uint8_t buf[64];
    static uint8_t pos = 0;
    buf[pos++] = b;

    /* un paquet complet ? [addr] [len] ... [crc] */
    if (pos >= 2 && pos >= buf[1] + 2) {

        /* 1)  DEVICE_PING  ->  DEVICE_INFO  */
        if (buf[0] == CRSF_ADDRESS_CRSF_TRANSMITTER &&
            buf[2] == CRSF_FRAMETYPE_DEVICE_PING) {         // 0x28
            sendDeviceInfo();                               // répond 0x29
        }

        pos = 0;                                            // reset
    }
    if (pos >= sizeof(buf)) pos = 0;

    /* Affiche quand même pour debug                    */
    dumpPacket(buf, pos);
}


/* =================================================================
 *                 Callbacks télémétrie (affichage console)
 * =================================================================*/
void onLinkStats(crsfLinkStatistics_t *ls)
{
    Serial.printf("[LINK] LQ:%u  RSSI1:%d  RSSI2:%d  SNR:%d  RF:%u\n",
                  ls->uplink_Link_quality,
                  ls->uplink_RSSI_1, ls->uplink_RSSI_2,
                  ls->uplink_SNR,
                  ls->rf_Mode);
}

void onGps(crsf_sensor_gps_t *gps)
{
    Serial.printf("[GPS] %.7f , %.7f  alt:%d  sats:%u\n",
                  gps->latitude  / 1e7,
                  gps->longitude / 1e7,
                  gps->altitude,
                  gps->satellites);
}

/* =================================================================
 *                               SETUP
 * =================================================================*/
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("Start");

    /* 1. UART inversé half‑duplex */
    CRSFPort.begin(CRSF_BAUD, SERIAL_8N1,
                   PIN_RX_OUT, PIN_TX_OUT,
                   true);                            // inversion
    //CRSFPort.setMode(UART_MODE_RS485_HALF_DUPLEX); // truc tout nul
    
    /* 2. Init lib + callbacks */
    crsf.begin();
    crsf.onOobData               = onOobByte;
    crsf.onPacketLinkStatistics  = onLinkStats;
    crsf.onPacketGps             = onGps;

    /* 3. Rafale « neutral » 200 ms pour sortir le RX du failsafe */
    uint32_t t0 = millis();
    while (millis() - t0 < 200) {   // ≃ 50 paquets à 250 Hz
        sendNeutralChannels();
        delayMicroseconds(SEND_INTERVAL_US);
    }
}

/* =================================================================
 *                               LOOP
 * =================================================================*/
void loop()
{
    static uint32_t lastSend = 0;

    crsf.loop();                                  // parse Rx

    if (micros() - lastSend >= SEND_INTERVAL_US) {
        sendSteerChannels();          // CH1 alterne gauche/droite
        lastSend = micros();
    }
}

void sendSteerChannels()
{
    static bool left = false;
    static uint32_t lastToggle = 0;

    // alterne toutes les secondes
    if (millis() - lastToggle >= 10000) {
        left = !left;
        lastToggle = millis();
    }
    uint16_t usCH = left ? 988 : 2012;     // gauche / droite

    /* 1.  mets tous les canaux à 1500 µs */
    for (uint8_t ch = 1; ch <= CRSF_NUM_CHANNELS; ++ch)
        crsf.setChannel(ch, 1500);

    /* 2.  channel 2 (index 1) = direction */
    crsf.setChannel(2, usCH);              // <- C'EST ICI !

    /* 3.  empaquette et envoie */
    crsf.queuePacketChannels();
}

/* ---------- Callback générique : hexdump de TOUT paquet --------- */
void dumpPacket(const uint8_t *buf, uint8_t len)
{
    Serial.print("RX: ");
    for (uint8_t i = 0; i < len; ++i) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        Serial.write(' ');
    }
    Serial.println();
}

// --- utilitaire : libère TX pendant 2 ms --------------------------
static inline void triStateTxLine()
{
    CRSFPort.flush();                  // attend fin d'envoi
    pinMode(PIN_TX_OUT, INPUT_PULLUP); // ligne libre
    delayMicroseconds(200);           // laisse le module parler
    pinMode(PIN_TX_OUT, OUTPUT);
    digitalWrite(PIN_TX_OUT, HIGH);   // niveau repos (inversé)
}
