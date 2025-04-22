#include "HID_Wheel.h"
#include <usbhub.h>
#include <SPI.h>
#include "CrsfSerial.h"
#include <HardwareSerial.h>

#define PIN_RX_OUT 2
#define PIN_TX_OUT 4
constexpr int        LED_PIN      = 13;       // LED de diagnostique
constexpr uint32_t   CRSF_BAUD    = 420000;   // 250 Hz ELRS

#define CRSF_VOLANT_MIN 200
#define CRSF_VOLANT_MAX 1500

#define CRSF_VITESSE_MIN 700
#define CRSF_VITESSE_MAX 1200

#define OFFSET_VOLANT 73
#define OFFSET_CAR 0
#define OFFSET_CAR_1_18 158

// Objets pour le shield USB
USB          Usb;
USBHub       Hub(&Usb);
HID_Wheel    wheel(&Usb);

// Objets Alfredo
HardwareSerial CRSFUart(1);                   // Serial1 (UART1)
CrsfSerial crsf(CRSFUart, CRSF_BAUD);

// Pour limiter à 250Hz
uint32_t lastTx = 0;
static unsigned long lastSend  = 0;
static unsigned long lastPrint = 0;
static uint8_t launched = 0;
static uint8_t bouing = 0;

// 60 02 : force globale
// 60 08 : angle de rotation

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

/* ---------- Callbacks (frames reçues) --------------------------- */
void onLinkStats(crsfLinkStatistics_t *ls)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));     // blink

    /*  Prend le meilleur RSSI des deux antennes si présent          */
    int8_t rssi = ls->uplink_RSSI_1;
    if (ls->uplink_RSSI_2)                 // vaut 0 si la seconde n’existe pas
        rssi = max(rssi, ls->uplink_RSSI_2);

    Serial.printf("[Link] LQ:%u%%  RSSI:%d  SNR:%d\n",
                  ls->uplink_Link_quality,
                  rssi,
                  ls->uplink_SNR);
}

struct __attribute__((packed)) crsf_device_info_t {
    uint8_t serial_number[4]   {0x12,0x34,0x56,0x78};
    uint8_t hardware_version   {1};
    uint8_t software_major     {0};
    uint8_t software_minor     {1};
    uint8_t reserved1          {0};
    uint8_t device_options     {0};
    uint8_t empty              {0};
};

void onOobByte(uint8_t b)
{
    static uint8_t buf[6];
    static uint8_t pos = 0;
    buf[pos++] = b;
    if (pos < 6) return;

    if (buf[0] == CRSF_ADDRESS_CRSF_TRANSMITTER && buf[2] == CRSF_FRAMETYPE_DEVICE_PING) {
        crsf_device_info_t info;
        crsf.queuePacket(CRSF_FRAMETYPE_DEVICE_INFO,
                         &info, sizeof(info));
    }
    pos = 0;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);                  
  while (!Serial) {
    // Attendre l'USB série
  }
  Serial.println("ESP Started");
  // Initialiser le shield USB
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
  }
  Serial.println("Waiting 5s for Radiomaster...");
  delay(5000);

  /* UART1 : inversion logique + half‑duplex -------------------- */
  CRSFUart.begin(CRSF_BAUD,
                 SERIAL_8N1,
                 PIN_RX_OUT,           // RX
                 PIN_TX_OUT,           // TX
                 true);                // invert
  crsf.onOobData = onOobByte;   // chaque octet « hors CRSF » arrive ici

  Serial.println("Serial  CRSF start");
  Serial.println(CRSF_BAUDRATE);

  /* Callbacks downlink ----------------------------------------- */
  crsf.onPacketLinkStatistics = onLinkStats;

  const uint8_t cmd_bind[3] = {0x01, 0x00, 0x00};      // sub‑cmd Bind
  crsf.queuePacket(CRSF_FRAMETYPE_COMMAND,              // 0x32
                   cmd_bind, sizeof(cmd_bind));
  delay(200);   // laisse le temps au module
 
 }

void loop() {
  // Gère la pile USB, déclenche Parse() quand un rapport arrive
  Usb.Task();
  crsf.loop();                            // gère les paquets reçus
    
  if (wheel.connected()) {
    // On limite à ~100Hz

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

    if (millis() - lastTx >= 4) {           // ≈ 250 Hz
      lastSend = millis();

      if (wheel.bouing_value == 1){
        sendBouing();
      }

      uint16_t volant_value = constrain(wheel.volant_value + OFFSET_CAR + OFFSET_VOLANT, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
      uint16_t vitesse_value = constrain(CRSF_CHANNEL_VALUE_MID + wheel.accel_value - wheel.frein_value, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);

    /* Envoyer les 16 canaux toutes les 4 ms ------------------- */
      crsf.setChannel(1, volant_value);   // CH1 = volant
      for (uint8_t ch = 2; ch <= 16; ++ch)
          crsf.setChannel(ch, 1500);      // neutre

      crsf.queuePacketChannels();         // push vers module
      lastTx = millis();
      
      if (millis() - lastPrint >= 400) {
        lastPrint = millis();
        Serial.print("crsfChannels.ch0 = ");
        Serial.println(volant_value);
      }
    }
  }
}

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
