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

enum class VideoStandard : uint8_t
{
    RESERVED = 0b000,
    NTSC_M_J = 0b001,    // (M, J) NTSC ITU-R BT.601
    PAL_BDGHI_N = 0b011, // PAL B,D,G,H,I,N
    PAL_M = 0b101,
    PAL_Nc = 0b111,
    NTSC_443 = 0b1001,
    SECAM = 0b1011
};

enum VideoOutputFormat
{
    DISCRETE_SYNC_YCBCR_422 = 0x00, // Uses HSYNC/VSYNC pins
    EMBEDDED_SYNC_BT656 = 0x07      // Uses SAV/EAV codes (No sync pins needed)
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
    static constexpr uint8_t TVP_REG_MISC_OUTPUT_CONTROLS = 0x05;

    static constexpr uint8_t TVP_REG_CONFIG_SHARED_PINS = 0x0F;

    static constexpr uint8_t TVP_REG_INTERRUPT_STATUS_A = 0xC0;

    static constexpr uint8_t TVP_REG_AVID_CROP_START_MSB = 0x11;
    static constexpr uint8_t TVP_REG_AVID_CROP_START_LSB = 0x12;
    static constexpr uint8_t TVP_REG_AVID_CROP_STOP_MSB = 0x13;
    static constexpr uint8_t TVP_REG_AVID_CROP_STOP_LSB = 0x14;
    static constexpr uint8_t TVP_REG_VBLK_CROP_START = 0x18;
    static constexpr uint8_t TVP_REG_VBLK_CROP_STOP = 0x19;

    static constexpr uint8_t TVP_REF_CB_GAIN_FACTOR = 0x2C;
    static constexpr uint8_t TVP_REF_CR_GAIN_FACTOR = 0x2D;

    static constexpr uint8_t TVP_DEVICE_ID_MSB = 0x80;
    static constexpr uint8_t TVP_DEVICE_ID_LSB = 0x81;

    static constexpr uint8_t TVP_VERTICAL_LINE_MSB = 0X84;
    static constexpr uint8_t TVP_VERTICAL_LINE_LSB = 0X85;

    static constexpr uint8_t TVP_OUTPUT_DATA_SELECT = 0X0D;

    // Status Registers

    static constexpr uint8_t TVP_REG_STATUS_ONE = 0x88;
    static constexpr uint8_t TVP_REG_STATUS_TWO = 0x89;
    static constexpr uint8_t TVP_REG_STATUS_THREE = 0x8A;
    static constexpr uint8_t TVP_REG_STATUS_FOUR = 0x8B;
    static constexpr uint8_t TVP_REG_STATUS_FIVE = 0x8C;

    // Gain_Step_Setting
    static constexpr uint8_t GAIN_STEP[16] = {
        61, 55, 48, 44, 38, 33, 29, 26,
        24, 22, 20, 19, 18, 17, 16, 15};

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

    // TODO: UNTESTED
    //------
    bool source_select(CAM_SELECT camera);
    //------

    // examples
    bool setup_ex_1_ntsc_to_bt656();

    // Writes

    // TODO: UNTESTED
    //------

    bool set_gpcl_logic_level(bool level);
    bool set_ycbcr_output_enable(bool enable_ycbcr_output);
    bool set_clock_output_enable(bool enable_clock);
    bool reset_miscellaneous_controls_register();
    bool set_output_format(VideoOutputFormat format);

    //------

    // TODO: UNTESTED
    //------

    bool set_crop_avid_horizontal(int16_t start, int16_t stop);
    bool set_crop_vblk_vertical(int8_t start_offset, int8_t stop_offset);
    bool set_avid_output_enable(bool enable);
    bool set_gpcl_or_vblk_output(bool enable_gpcl_output);
    bool set_avid_out_active_during_vblk(bool active);
    bool reset_crop();

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
    uint16_t read_vertical_line_count();

    //------

    // TODO Status Register 2: UNTESTED
    //------

    bool read_weak_signal();
    bool read_field_sequence_status();
    bool read_AGC_frozen_status();

    //------

    // TODO Status Register 3: UNTESTED
    //------

    uint8_t read_analog_gain();
    uint8_t read_digital_gain();
    uint8_t read_gain_product();

    //------

    // TODO Status Register 5: UNTESTED
    //------

    bool read_autoswitch_mode();
    VideoStandard read_video_standard();

    //------
};
