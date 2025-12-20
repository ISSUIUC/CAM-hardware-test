#include <Wire.h>
#include <Arduino.h>

enum CAM_SELECT
{
    CAM1 = 0,
    CAM2 = 1
};

// Device registers
#define TVP_DEVICE_ID_MSB 0x80
#define TVP_DEVICE_ID_LSB 0x81
#define TVP_MISC_CONTROLS 0x03
#define TVP_CONFIG_SHARED_PINS 0x0F

class tvp5151
{
private:
    TwoWire *_i2c;
    uint8_t _i2c_addr;
    uint8_t _pdn;
    uint8_t _reset;

    uint8_t read_register(uint8_t address);

public:
    tvp5151(uint8_t pdn, uint8_t reset, uint8_t i2c_addr, TwoWire *i2c);

    bool init();
    void source_select(CAM_SELECT camera);

    /* Toggles the `GPCL (general purpose control logic)/VBLK (vertical blanking)` output, set_enabled=TRUE will enable GPCL output, otherwise VBLK will be selected
        - Should enable the INTREQ/GPCL/VBLK output enable, bit 5 of 0x03h
    */
    void en_gpcl_output(bool set_enabled);
    void toggle_gpcl_logic_level(bool level);

    // examples
    void setup_ex_1_ntsc_to_bt656();

    uint16_t read_device_id();
};
