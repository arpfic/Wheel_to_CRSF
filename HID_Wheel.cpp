#include "HID_Wheel.h"
#include <Arduino.h> // Pour Serial, map(), etc.

void printSectionBin(uint8_t *buf, size_t len, uint8_t begin, uint8_t end) {
    // Sécurité : vérifier les bornes
    if (begin >= len || end > len || begin >= end) {
        Serial.println("Paramètres invalides");
        return;
    }

    for (size_t i = begin; i < end; i++) {
        for (int j = 7; j >= 0; j--) {
            Serial.print((buf[i] >> j) & 1);
        }
        Serial.print(" ");
    }
    Serial.println();
}

void HID_Wheel::ParseHIDData(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  // 1) Vérifier si on est sur un device supporté
  if (!isThrustmasterNew() && !isThrustmasterOld() && !isLogitech()) {
      Serial.println("Volant non connecté/supporté");
      return; // On ignore
  }

  // Selon le modèle, on applique un parseur différent
  if (isThrustmasterNew()) {
    // On suppose qu'il envoie au moins 64 octets, etc.
    if (len < 64) return;

    // Volant sur 16 bits (o44 MSB, o43 LSB)
    uint16_t volant_raw = ((uint16_t)buf[44] << 8) | buf[43];
    // Mapping -> 1500..250
    volant_value = map(volant_raw, 0, 65535, 2000, 1000);

    // Accel. sur 8 bits (o46)
    uint16_t accel_raw = (uint16_t)buf[46];
    accel_value = map(accel_raw, 255, 0, 0, 500);

    // Volant sur 16 bits (o48 MSB, o47 LSB)
    uint16_t frein_raw = ((uint16_t)buf[48] << 8) | buf[47];
    frein_value = map(frein_raw, 65535, 0, 0, 500);

    // Bouing !
    if (buf[9] == 0xFF) {
      bouing_value = 1;
    } else {
      bouing_value = 0;
    }

    //raw
    //printSectionBin(buf, len, 46, 50);
    //Serial.print("Trustmaster New : ");
    //Serial.print("  Volant=");
    //Serial.print(volant_value);
    //Serial.print("  Accel=");
    //Serial.print(accel_value);
    //Serial.print("  Frein=");
    //Serial.println(frein_value);
  }
  else if (isThrustmasterOld()) {
    // On suppose qu'il envoie 27 octets, etc.
    if (len < 27) return;

    // Volant sur 12 bits (o2, o3)
    uint16_t volant_raw = ((uint16_t)buf[3] << 8) | buf[2];
    // On retire l'offset du milieu
    if (volant_raw < 32783) {
      volant_raw = volant_raw + 9792;
    }
    if (volant_raw > 32783) {
      volant_raw = volant_raw - 9792;
    }
    // Mapping -> 1500..250
    volant_value = map(volant_raw, 9807, 55743, 1500, 250);

    // Accel. sur 8 bits (o18)
    uint16_t accel_raw = (uint16_t)buf[18];
    accel_value = map(accel_raw, 0, 255, 0, 800);

    // Frein sur 8 bits (o17)
    uint16_t frein_raw = (uint16_t)buf[17];
    frein_value = map(frein_raw, 0, 255, 0, 820);

    //raw
    //printSectionBin(buf, len, 43, 45);
    /*
    Serial.print("Trustmaster Old : ");
    Serial.print("  Volant=");
    Serial.print(volant_value);
    Serial.print("  Accel=");
    Serial.print(accel_value);
    Serial.print("  Frein=");
    Serial.println(frein_value);
    */
  }
  else if (isLogitech()) {
    // On suppose qu'il envoie 7 octets, etc.
    if (len < 7) return;

    // Volant sur 10 bits (o0, o1)
    uint16_t volant_raw = ((uint16_t)(buf[1] & 0x03) << 8) | buf[0];

    // Mappage inversé 0..1023 -> 1500..250
    volant_value = map(volant_raw, 0, 1023, 1500, 250);

    // Pédale : bits 1..7 de buf[3], plus buf[5]/buf[6], etc.
    // Cf. votre logique d'accel/frein
    if (buf[3] & 0x80) {
        // On set accel=0, parse frein
        accel_value = 0;
        uint8_t sevenBits = (buf[3] >> 1) & 0x7F;
        uint16_t frein_raw = ((uint16_t)sevenBits << 8) | buf[6];
        // On retire l'offset
        frein_raw = frein_raw - 16383;
        frein_value = map(frein_raw, 0, 16383, 0, 625);
    } else {
        // On set frein=0, parse accel
        frein_value = 0;
        uint8_t sevenBits = (buf[3] >> 1) & 0x7F;
        uint16_t accel_raw = ((uint16_t)sevenBits << 8) | buf[5];
        accel_raw = 16383 - accel_raw;
        accel_value = map(accel_raw, 0, 16383, 0, 625);
    }
//    Serial.print("Logitech : ");
//    Serial.print("  Volant=");
//    Serial.print(volant_value);
//    Serial.print("  Accel=");
//    Serial.print(accel_value);
//    Serial.print("  Frein=");
//    Serial.println(frein_value);
  }
}

// Helpers
