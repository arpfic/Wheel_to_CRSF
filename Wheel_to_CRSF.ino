#include "HID_Wheel.h"
#include <usbhub.h>
#include <SPI.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX_OUT 2
#define PIN_TX_OUT 4

#define CRSF_CHAN_MIN 172
#define CRSF_CHAN_MAX 1792

#define CRSF_VOLANT_MIN 200
#define CRSF_VOLANT_MAX 1500

#define CRSF_VITESSE_MIN 700
#define CRSF_VITESSE_MAX 1200

#define OFFSET_CAR_1_18 158

// Objets pour le shield USB
USB          Usb;
USBHub       Hub(&Usb);
HID_Wheel    wheel(&Usb);

// Objets Alfredo
HardwareSerial crsfSerialOut(2);
AlfredoCRSF crsfOut;
// Pour limiter à 100Hz
static unsigned long lastSend  = 0;
static unsigned long lastPrint = 0;

void setup() {
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

  crsfSerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_OUT, PIN_TX_OUT);  
  crsfOut.begin(crsfSerialOut);  
  Serial.println("Serial  CRSF start");
  Serial.println(CRSF_BAUDRATE);

  // burst - init des gaz
  for (int i = 0; i < 200; i++){
    sendFallbackChannels();
    delay(5);
  }
  Serial.println("Radiomaster ok...");
 }

void loop() {
  // Gère la pile USB, déclenche Parse() quand un rapport arrive
  Usb.Task();

  if (wheel.connected()) {
    // On limite à ~100Hz
    if (millis() - lastSend >= 10) {
      lastSend = millis();

      uint16_t volant_value = constrain(wheel.volant_value + OFFSET_CAR_1_18, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
      uint16_t vitesse_value = constrain(CRSF_CHANNEL_VALUE_MID + wheel.accel_value - wheel.frein_value, CRSF_VITESSE_MIN, CRSF_VITESSE_MAX);

      crsf_channels_t crsfChannels = { 0 };
      crsfChannels.ch0 = volant_value;
      crsfChannels.ch1 = vitesse_value;
      crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch4 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch5 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch6 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch7 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch8 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch9 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch10 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch11 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch12 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch13 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch14 = CRSF_CHANNEL_VALUE_MID;
      crsfChannels.ch15 = CRSF_CHANNEL_VALUE_MID;
      crsfOut.writePacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));

      // 5Hz
//      if (millis() - lastPrint >= 200) {
//        lastPrint = millis();
//        Serial.print("crsfChannels.ch0 = ");
//        Serial.print(volant_value);
//        Serial.print(" | crsfChannels.ch1 = ");
//        Serial.println(CRSF_CHANNEL_VALUE_MID + wheel.accel_value - wheel.frein_value);
//      }

    }
  }
}

// Method to send channels based on CRSF instance
void sendChannels(AlfredoCRSF& crsf) {
  const crsf_channels_t* channels_ptr = crsf.getChannelsPacked();
  crsfOut.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, channels_ptr, sizeof(*channels_ptr));
}

// Fallback method to send default channel values
void sendFallbackChannels() {
    crsf_channels_t crsfChannels = { 0 };
    crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch4 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch5 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch6 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch7 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch8 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch9 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch10 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch11 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch12 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch13 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch14 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch15 = CRSF_CHANNEL_VALUE_MID;
    crsfOut.writePacket(CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}
