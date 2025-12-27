#include <Wire.h>
#include <Arduino.h>

// Type defs
enum CAM_SELECT
{
    CAM1 = 0,
    CAM2 = 1
};

enum TVP_I2C_ERROR
{
    SUCCESS = 0,
    BUFFER_OVERFLOW = 1,
    NACK_ADDRESS = 2,
    NACK_DATA = 3,
    OTHER = 4,
    TIMEOUT = 5
};

typedef struct
{
    uint8_t data;
    TVP_I2C_ERROR result;
} tvp_i2c_result_t;

class tvp5151
{
private:
    // Device registers
    static constexpr uint8_t TVP_INPUT_SOURCE_SELECTION = 0x00;
    static constexpr uint8_t TVP_REG_MISC_CONTROLS = 0x03;
    static constexpr uint8_t TVP_REG_CONFIG_SHARED_PINS = 0x0F;

    static constexpr uint8_t TVP_REF_CB_GAIN_FACTOR = 0x2C;
    static constexpr uint8_t TVP_REF_CR_GAIN_FACTOR = 0x2D;

    static constexpr uint8_t TVP_DEVICE_ID_MSB = 0x80;
    static constexpr uint8_t TVP_DEVICE_ID_LSB = 0x81;

    // Utilites
    tvp_i2c_result_t read_register(uint8_t register_addr);
    void write_register(uint8_t register_addr, uint8_t data);
    void print_I2C_error(TVP_I2C_ERROR error);
    TVP_I2C_ERROR categorize_error(uint8_t error);

    // other variables
    TwoWire *_i2c;
    uint8_t _i2c_addr;
    uint8_t _pdn;
    uint8_t _reset;

public:
    tvp5151(uint8_t pdn, uint8_t reset, uint8_t i2c_addr, TwoWire *i2c);

    bool init();
    uint16_t read_device_id();
    void source_select(CAM_SELECT camera);

    bool en_gpcl_output(bool set_enabled);
    bool toggle_gpcl_logic_level(bool level);

    // examples
    void setup_ex_1_ntsc_to_bt656();

    uint16_t read_device_id();

    uint8_t read_cb_gain();
    uint8_t read_cr_gain();
};
