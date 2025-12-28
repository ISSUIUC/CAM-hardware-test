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
    static constexpr uint8_t TVP_REG_STATUS_ONE = 0x88;
    static constexpr uint8_t TVP_REG_INTERRUPT_STATUS_A = 0xC0;

    static constexpr uint8_t TVP_REF_CB_GAIN_FACTOR = 0x2C;
    static constexpr uint8_t TVP_REF_CR_GAIN_FACTOR = 0x2D;

    static constexpr uint8_t TVP_DEVICE_ID_MSB = 0x80;
    static constexpr uint8_t TVP_DEVICE_ID_LSB = 0x81;

    // Utilites
    tvp_i2c_result_t read_register(uint8_t register_addr);
    bool write_register(uint8_t register_addr, uint8_t data);

    void print_I2C_error(TVP_I2C_ERROR error);
    TVP_I2C_ERROR categorize_error(uint8_t error);

    // TODO: Untested
    //------

    bool modify_register_bit(uint8_t reg, uint8_t bit_mask, bool state);
    bool read_register_bit(uint8_t reg, uint8_t bit_mask);

    //------

    // other variables
    TwoWire *_i2c;
    uint8_t _i2c_addr;
    uint8_t _pdn;
    uint8_t _reset;

public:
    tvp5151(uint8_t pdn, uint8_t reset, uint8_t i2c_addr, TwoWire *i2c);

    // Bare-bone stuff
    bool init();
    uint16_t read_device_id();
    void source_select(CAM_SELECT camera);

    // examples
    void setup_ex_1_ntsc_to_bt656();

    // Writes

    // TODO: UNTESTED
    //------

    bool set_gpcl_output(bool enable_gpcl_output);
    bool set_gpcl_logic_level(bool level);
    void reset_miscellaneous_controls_register();
    bool set_ycbcr_output_enable(bool enable_ycbcr_output);
    bool set_clock_output_enable(bool enable_clock);

    //------

    // Reads
    uint8_t read_cb_gain();
    uint8_t read_cr_gain();

    // TODO: UNTESTED
    //------

    bool read_vertical_sync_lock_status();
    bool read_horizontal_sync_lock_status();
    bool read_peak_white_detect_status();
    bool read_lock_state_interrupt();
    bool read_vcr_mode();
    bool read_lost_lock_status();
    bool read_color_subcarrier_lock_status();

    //------
};
