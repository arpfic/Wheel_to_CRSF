#ifndef _HID_WHEEL_H_
#define _HID_WHEEL_H_

#include <hiduniversal.h>
#include <AlfredoCRSF.h>

// USB IDs
#define VID_THRUSTMASTER  0x044F
#define PID_TH_WHEEL_OLD  0xB65D
#define PID_TH_WHEEL_NEW  0XB681

#define VID_LOGITECH      0x046D
#define PID_LOGI_WHEEL    0xC294

/**
 * Classe générique "HID_Wheel"
 * - Dérive de HIDUniversal pour accéder à VID/PID protégés
 * - Gère la détection de plusieurs modèles (Thrustmaster, Logitech, etc.)
 * - Stocke quelques valeurs d'axes en variables membres
 */
class HID_Wheel : public HIDUniversal {
public:
    HID_Wheel(USB *usb) : HIDUniversal(usb) {}

    // Vérifie si c'est "Ready" et si on reconnaît un volant supporté
    bool connected() {
        return isReady() && (isThrustmasterNew() || isThrustmasterOld() || isLogitech());
    }

    // Variables publiques où l'on stocke les mesures (exemple)
    uint16_t volant_value = CRSF_CHANNEL_VALUE_MID;
    uint16_t accel_value  = 0;
    uint16_t frein_value  = 0;

protected:
    bool isThrustmasterNew() {
        return (VID == VID_THRUSTMASTER && PID == PID_TH_WHEEL_NEW);
    }

    bool isThrustmasterOld() {
        return (VID == VID_THRUSTMASTER && PID == PID_TH_WHEEL_OLD);
    }

    bool isLogitech() {
        return (VID == VID_LOGITECH && PID == PID_LOGI_WHEEL);
    }

    // Surcharges pour la dérivation HIDUniversal

    virtual void ParseHIDData(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) override;
    
    virtual uint8_t OnInitSuccessful() override {
        // Test si on reconnaît l'ID du device
        if (!isThrustmasterNew() && !isThrustmasterOld() && !isLogitech()) {
            return 1; // Pas un device supporté, "refus"
        }
        Serial.println("HID_Wheel init successful!");
        return 0; // OK
    }
};

#endif
