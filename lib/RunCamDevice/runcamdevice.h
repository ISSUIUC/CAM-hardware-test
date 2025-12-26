#pragma once
#include <Arduino.h>
#include <runcam_crc.h>

template <uint8_t... Bytes>
constexpr auto rc_command() {
    constexpr uint8_t raw[] = { Bytes... };
    return std::array<uint8_t, sizeof...(Bytes) + 1>{
        Bytes..., rc_generate_crc(raw, sizeof...(Bytes))
    };
}

class RCDevice {
    protected:
    HardwareSerial* _ser;
    uint8_t _tx = 0;
    uint8_t _rx = 0;


    public:
    RCDevice(HardwareSerial& serial) {_ser = &serial;};
    RCDevice(HardwareSerial& serial, uint8_t tx, uint8_t rx) {
        _ser = &serial;
        _tx = tx;
        _rx = rx;
    }
    void init() {
        if (_tx == _rx) { _ser->begin(115200); }
        _ser->begin(115200, SERIAL_8N1, _rx, _tx);
    }
};

class RCDevice2Button : RCDevice {
    /* Describes a set of runcam devices which use the 2-button (Non-OSD) protocol. Mostly seen on split type cameras. */
    bool on_off();
};

using RCSplit4v2 = RCDevice2Button;


