#include <tvp5151.h>

tvp5151::tvp5151(uint8_t pdn, uint8_t reset, uint8_t addr, TwoWire *i2c)
{
    _i2c_addr = addr;
    _i2c = i2c;
    _pdn = pdn;
    _reset = reset;
}

bool tvp5151::init()
{
    // Page 23 - Section 3.18
    pinMode(_pdn, OUTPUT);
    pinMode(_reset, OUTPUT);

    digitalWrite(_pdn, HIGH);
    delay(20);

    digitalWrite(_reset, LOW);
    delayMicroseconds(500);
    digitalWrite(_reset, HIGH);
    delayMicroseconds(250);

    return true;
}

// 3.21.40 MSB of Device ID Register
// 3.21.41 LSB of Device ID Register

uint16_t tvp5151::read_device_id()
{

    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_DEVICE_ID_MSB);
    uint8_t error = _i2c->endTransmission(true);
    if (error != SUCCESS)
    {
        Serial.println("[ERROR] I2C read_device_id reading error");
        print_I2C_error(categorize_error(error));
        return -1;
    }

    _i2c->requestFrom(_i2c_addr, 2);
    uint8_t id_lsb = _i2c->read();
    uint8_t id_msb = _i2c->read();
    uint16_t device_id = (id_msb << 8) + id_lsb;

    return device_id;
}

// for the registers should I have a boolean for each bit such as black output if I want it on or should this just select camera in which it's decoding? I think the latter
bool tvp5151::source_select(CAM_SELECT CAM)
{

    if (CAM)
    { // CAM1 (AIP1B)
        return write_register(TVP_INPUT_SOURCE_SELECTION, 0x02);
    }
    else
    { // CAM0 (AIP1A)
        return write_register(TVP_INPUT_SOURCE_SELECTION, 0x00);
    }
}

bool tvp5151::setup_ex_1_ntsc_to_bt656()
{
    // Sets registers according to the default spec 1
    return write_register(TVP_REG_MISC_CONTROLS, 0x09); // 0x09 Enables clock & YCbCr output
}

//----------------------------------------------------------------------------------------------------------------------------------

// Outputs and Data Rates Select Register
// Extended Range vs BT.601 Limited Range (We are using default (Extended code range) and
// if video dosen't look right it's said that this is a common error with video so could be worth checking into more)
// ESP32 / FPGA processing Reccomends extended range so this is what is set by default

// 3.21.16 Active Video Cropping Start Pixel MSB Register
// We are not doing any sort of AVID right? Because our CAM Signal alligns with video decoder standard 720 x 480 ?

// 3.21.20 Genlock and RTC Register
// Real Time Communcation I need to look into this more and if we need it as we are ultimately wanting to stream live video? Does it correlate at all?

// 3.21.33 Cb Gain Factor Register
uint8_t tvp5151::read_cb_gain()
{
    tvp_i2c_result_t reg_val = read_register(TVP_REF_CB_GAIN_FACTOR);
    if (reg_val.result == SUCCESS)
    {
        return reg_val.data;
    }
    return -1;
}

uint8_t tvp5151::read_cr_gain()
{
    tvp_i2c_result_t reg_val = read_register(TVP_REF_CR_GAIN_FACTOR);
    if (reg_val.result == SUCCESS)
    {
        return reg_val.data;
    }
    return -1;
}

/*
3.21.61 Interrupt Status Register A | Bit 7 (Page 62)
"The interrupt status register A can be polled by the host processor to determine the source of an interrupt.
After an interrupt condition is set it can be reset by writing to this register with a 1 in the appropriate bit(s)."
^ Basically, to acknowledge the interrupt we need to write a 1 to this place (this function does that automatically)
Lock state interrupt
0 = TVP5151 is not locked to the video signal (default).
1 = TVP5151 is locked to the video signal.

This is essentially an AND statement comparing the status of the vertical, hori, and color locks.
*/
bool tvp5151::read_lock_state_interrupt()
{
    bool ret = read_register_bit(TVP_REG_INTERRUPT_STATUS_A, 0x80);
    if (!write_register(TVP_REG_INTERRUPT_STATUS_A, 0x80))
        return false;
    return ret;
}

/*
3.21.48 Status Register #1 | Bit 2 (Page 54)
Vertical sync lock status
0 = Vertical sync is not locked.
1 = Vertical sync is locked.
Returns 1 if the chip is successfully tracking the vertical timing (frames)
*/
bool tvp5151::read_vertical_sync_lock_status()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x04);
}

/*
3.21.48 Status Register #1 | Bit 1 (Page 54)
Horizontal sync lock status
0 = Horizontal sync is not locked.
1 = Horizontal sync is locked.
Returns 1 if the chip is successfully tracking the horizontal timing (lines)
*/
bool tvp5151::read_horizontal_sync_lock_status()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x02);
}

/*
3.21.48 Status Register #1 | Bit 3 (Page 54)
Color subcarrier lock status
0 = Color subcarrier is not locked.
1 = Color subcarrier is locked.
*/
bool tvp5151::read_color_subcarrier_lock_status()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x08);
}

/*
3.21.48 Status Register #1 | Bit 4 (Page 54)
Lost lock detect
0 = No lost lock since status register #1 was last read.
1 = Lost lock since status register #1 was last read.
It returns 1 if the lock was dropped at any point since you last read this register
Reading the register automatically resets this bit to 0 (nice)
*/
bool tvp5151::read_lost_lock_status()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x10);
}

/*
3.21.48 Status Register #1 | Bit 1 (Page 54)
Horizontal sync lock status
0 = Horizontal sync is not locked.
1 = Horizontal sync is locked.
*/
bool tvp5151::read_peak_white_detect_status()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x80);
}

/*
3.21.48 Status Register #1 | Bit 0 (Page 54)
TV/VCR status. TV mode is determined by detecting standard line-to-line variations and specific
chrominance SCH phases based on the standard input video format. VCR mode is determined by
detecting variations in the chrominance SCH phases compared to the chrominance SCH phases of the
standard input video format.
0 = TV
1 = VCR
*/
bool tvp5151::read_vcr_mode()
{
    return read_register_bit(TVP_REG_STATUS_ONE, 0x01);
}



// 3.21.49 Status Register #2

// 3.21.49 Status Register #2 || Bit 6 
// Weak signal detection
// 0 = No weak signal
// 1 = Weak signal mode
bool tvp5151::read_weak_signal(){
    return read_register_bit(TVP_REG_STATUS_TWO, 0x40);
}

// 3.21.49 Status Register #2 || Bit 4 
// Field sequence status
// 0 = Even field (False)
// 1 = Odd field (True)
bool tvp5151::read_field_sequence_status(){
    return read_register_bit(TVP_REG_STATUS_TWO, 0x10);
}


// 3.21.49 Status Register #2 || Bit 3
// AGC and offset frozen status
// 0 = AGC and offset are not frozen.
// 1 = AGC and offset are frozen.
bool tvp5151::read_AGC_frozen_status(){
    return read_register_bit(TVP_REG_STATUS_TWO, 0x08);
}



// 3.21.50 Status Register #3

// 3.21.50 Status Register #3 || BIT 4-8 
// Analog gain: 4-bit front-end AGC analog gain setting
// 0 ≤ analog_gain ≤ 15
uint8_t tvp5151::read_analog_gain(){
    tvp_i2c_result_t reg = read_register(TVP_REG_STATUS_THREE);
    uint8_t analog_gain = reg.data &= 0xF0;
    return analog_gain; 

}
// 3.21.50 Status Register #3 || BIT 0-3
// Digital gain: 4 MSBs of 6-bit front-end AGC digital gain setting
// 0 ≤ digital_gain ≤ 63
uint8_t tvp5151::read_digital_gain(){
    tvp_i2c_result_t reg = read_register(TVP_REG_STATUS_THREE);
    uint8_t digital_gain = reg.data &= 0x0F;
    return digital_gain; 
}
 

// 3.21.50 Status Register #3 
// The product of the analog and digital gain is as follows: 
// Gain Product = (1 + 3 × analog_gain / 15) × (1 + gain_step × digital_gain / 4096)
// The gain_step setting as a function of the analog_gain setting is shown in Table 3-15.
uint8_t tvp5151::read_gain_product(){
    uint8_t digital_gain = read_digital_gain();
    uint8_t analog_gain = read_analog_gain();
    uint8_t gain_product = (1+(3*(analog_gain/15))) * (1 + ((GAIN_STEP[analog_gain] * digital_gain)/4096));
    return gain_product; 
    
}


// 3.21.51 Status Register #4


// Subcarrier to horizontal (SCH) phase (I don't think we need it)


// 3.21.52 Status Register #5
// This register contains information about the detected video standard at which the device is currently
// operating. When autoswitch code is running, this register must be tested to determine which video
// standard has been detected.


// 3.21.52 Status Register #5 || Bit 7
bool tvp5151::read_autoswitch_mode(){   
    return read_register_bit(TVP_REG_STATUS_FIVE, 0x80);
}


// 3.21.52 Status Register #5 || Bit 0-3
VideoStandard tvp5151::read_video_standard(){
    tvp_i2c_result_t value = read_register(TVP_REG_STATUS_FIVE);

    uint8_t video_bit_val = value.data &= 0x0F;

    switch(video_bit_val){
        case 0b001: return VideoStandard::NTSC_M_J;
        case 0b011: return VideoStandard::PAL_BDGHI_N;
        case 0b101: return VideoStandard::PAL_M;
        case 0b111: return VideoStandard::PAL_Nc;
        case 0b1001: return VideoStandard::NTSC_443;
        case 0b1011: return VideoStandard::SECAM;
        default: return VideoStandard::RESERVED; 
    }
}





//--------------------------------------------------------------------------------------------------------------

bool tvp5151::reset_miscellaneous_controls_register()
{
    return write_register(TVP_REG_MISC_CONTROLS, 0x00);
}

/*
3.21.4 Miscellaneous Controls Register | Bit 0
Clock output enable
0 = SCLK output is high impedance (default)
1 = SCLK output is enabled
*/
bool tvp5151::set_clock_output_enable(bool enable)
{
    return modify_register_bit(TVP_REG_MISC_CONTROLS, 0x01, enable);
}

/*
3.21.4 Miscellaneous Controls Register | Bit 3
YCbCr output enable
0 = YOUT[7:0] high impedance (default)
1 = YOUT[7:0] active
*/
bool tvp5151::set_ycbcr_output_enable(bool enable_ycbcr_output)
{
    return modify_register_bit(TVP_REG_MISC_CONTROLS, 0x08, enable_ycbcr_output);
}

/*
3.21.4 Miscellaneous Controls Register | Bit 6
GPCL logic level (affects INTREQ/GPCL/VBLK output only if bit 7 is set to 0 and bit 5 is set to 1)
0 = GPCL is set to logic 0 (default)
1 = GPCL is set to logic 1
*/
bool tvp5151::set_gpcl_logic_level(bool level)
{
    return modify_register_bit(TVP_REG_MISC_CONTROLS, 0x40, level);
}

/*
3.21.4 Miscellaneous Controls Register | Bit 5 & 7(Page 31)
Bit 5: INTREQ/GPCL/VBLK output enable
0 = Output disabled (default)
1 = Output enabled (recommended)
Bit 7:
VBLK/GPCL function select (affects INTREQ/GPCL/VBLK output only if bit 1 of I2C register 0Fh is set to
1)
0 = GPCL (default)
1 = VBLK

3.21.15 Configuration Shared Pins Register | Bit 1 (Page 40)
INTREQ/GPCL/VBLK (pin 27) function select
0 = INTREQ (default)
1 = GPCL or VBLK depending on bit 7 of register 03h
*/
bool tvp5151::set_gpcl_output(bool enable_gpcl_output)
{
    // Select GPCL function in the shared pin register
    if (!write_register(TVP_REG_CONFIG_SHARED_PINS, 0x02))
        return false;

    // Enable the output driver (Bit 5)
    if (!modify_register_bit(TVP_REG_MISC_CONTROLS, 0x20, true))
        return false;

    // Set Bit 7: 0 for GPCL, 1 for VBLK
    return modify_register_bit(TVP_REG_MISC_CONTROLS, 0x80, !enable_gpcl_output);
}



// 3.21.44 Vertical Line Count MSB Register 
// 3.21.45 Vertical Line Count LSB Register (Used for seeing how many lines per frame)
uint16_t tvp5151::read_vertical_line_count(){
    tvp_i2c_result_t read_reg_1 = read_register(TVP_VERTICAL_LINE_MSB);
    uint8_t MSB_Bit = read_reg_1.data &= 0x03;
    tvp_i2c_result_t read_reg_2 = read_register(TVP_VERTICAL_LINE_LSB);
    uint8_t LSB_Bit = read_reg_2.data; 
    uint16_t vertical_line_count = (MSB_Bit<<8) + LSB_Bit;
    return vertical_line_count;

}




//--------------------------------------------------------------------------------------------------------------

tvp_i2c_result_t tvp5151::read_register(uint8_t register_addr)
{
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(register_addr);
    tvp_i2c_result_t reg_val;

    uint8_t error = _i2c->endTransmission(true);
    reg_val.result = categorize_error(error);

    if (error)
    {
        Serial.println("[ERROR] I2C read failed on read_register");
        print_I2C_error(reg_val.result);
        return reg_val;
    }

    _i2c->requestFrom(_i2c_addr, 1);
    uint8_t register_data = _i2c->read();
    reg_val.data = register_data;
    return reg_val;
}

bool tvp5151::write_register(uint8_t register_addr, uint8_t data)
{
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(register_addr);
    _i2c->write(data);
    uint8_t error = _i2c->endTransmission(true);
    if (error)
    {
        Serial.println("[ERROR] I2C read failed on write_register");
        print_I2C_error(categorize_error(error));
        return false;
    }
    return true;
}

bool tvp5151::modify_register_bit(uint8_t reg, uint8_t bit_mask, bool state)
{
    tvp_i2c_result_t reg_read = read_register(reg);
    if (reg_read.result != SUCCESS)
        return false;

    uint8_t data = reg_read.data;
    if (state)
        data |= bit_mask;
    else
        data &= ~bit_mask;

    if (!write_register(reg, data))
        return false;

    return true;
}

/*
True - bit is 1.
False - bit is 0.
*/
bool tvp5151::read_register_bit(uint8_t reg, uint8_t bit_mask)
{
    tvp_i2c_result_t res = read_register(reg);

    if (res.result != SUCCESS)
    {
        Serial.println("[ERROR] I2C Read Error");
        print_I2C_error(categorize_error(res.result));
        return false;
    }

    // AND
    return (res.data & bit_mask) != 0;
}

void tvp5151::print_I2C_error(TVP_I2C_ERROR error)
{
    switch (error)
    {
    case SUCCESS:
        Serial.print("Success");
        break;
    case BUFFER_OVERFLOW:
        Serial.print("Buffer Overflow");
        break;
    case NACK_ADDRESS:
        Serial.print("No acknowledge on address");
        break;
    case NACK_DATA:
        Serial.print("No acknowledge on data");
        break;
    case OTHER:
        Serial.print("Other");
        break;
    case TIMEOUT:
        Serial.print("Time_out");
        break;
    }
}

TVP_I2C_ERROR tvp5151::categorize_error(uint8_t error)
{
    switch (error)
    {
    case 0:
        return SUCCESS;
    case 1:
        return BUFFER_OVERFLOW;
    case 2:
        return NACK_ADDRESS;
    case 3:
        return NACK_DATA;
    case 4:
        return OTHER;
    case 5:
        return TIMEOUT;
    }
}
