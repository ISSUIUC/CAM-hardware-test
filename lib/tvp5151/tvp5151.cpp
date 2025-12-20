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

uint16_t tvp5151::read_device_id()
{
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_DEVICE_ID_MSB);
    uint8_t err = _i2c->endTransmission(true);
    if (err)
    {
        Serial.println("I2C read failed on read_device_id");
        Serial.print("ERRNO ");
        Serial.println(err);
        return -1;
    }
    _i2c->requestFrom(_i2c_addr, 2);
    uint8_t id_lsb = _i2c->read();
    uint8_t id_msb = _i2c->read();

    uint16_t device_id = (id_msb << 8) + id_lsb;
    return device_id;
}

void tvp5151::en_gpcl_output(bool set_enabled)
{
    // Page 40 is for the  configuration shared pins register
    // Address is 0Fh, and default is 00h
    // We need to set bit 1 to be true so we write 0x02 to that address
    // this i sbecause this toggles the function of the GPL/INREQ/VBLK pin
    // 0 is INTREQ (defuault)
    // 1 is GPCL or VBLK depending on bit 7 of register 03h
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_CONFIG_SHARED_PINS);
    _i2c->write(0x02);
    _i2c->endTransmission();

    // page 31 has miscellanous controls register
    /*
    BIT 7:
    VBLK/GPCL function select (affects INTREQ/GPCL/VBLK output only if bit 1 of I2C register 0Fh is set to 1)
    0 = GPCL (default)
    1 = VBLK

    BIT 6:
    GPCL logic level (affects INTREQ/GPCL/VBLK output only if bit 7 is set to 0 and bit 5 is set to 1)
    0 = GPCL is set to logic 0 (default)
    1 = GPCL is set to logic 1 (HIGH)

    BIT 5:
    INTREQ/GPCL/VBLK output enable
    0 = Output disabled (default)
    1 = Output enabled (recommended)
    */

    uint8_t reg_val = read_register(TVP_MISC_CONTROLS);

    // So we gotta write 1 for bit 5 (0x20) and depending on set_enabled
    reg_val |= 0x20;

    if (set_enabled)
    {
        reg_val &= ~0x80; // set bit 7 low
    }
    else
    {
        reg_val |= 0x80; // bit 7 high
    }

    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_MISC_CONTROLS);
    _i2c->write(reg_val);
    _i2c->endTransmission();
}

void tvp5151::toggle_gpcl_logic_level(bool level)
{

    // read the register
    uint8_t reg_val = read_register(TVP_MISC_CONTROLS);

    if (level)
    {
        reg_val |= 0x40; // high
    }
    else
    {
        reg_val &= ~0x40; // Lowwww
    }

    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_MISC_CONTROLS);
    _i2c->write(reg_val);
    _i2c->endTransmission();
}

// Sets up NTSC to Bt.656 w/ YCbCr
void tvp5151::setup_ex_1_ntsc_to_bt656()
{
    // Register 03h
    //  Bit 3:
    /*
    YCbCr output enable
    0 = YOUT[7:0] high impedance (default)
    1 = YOUT[7:0] active
    */
    // Bit 0:
    /*
    Clock output enable
    0 = SCLK output is high impedance (default)
    1 = SCLK output is enabled
    */

    // 0x09 -> 0000 1001 (what we need)
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_MISC_CONTROLS);
    _i2c->write(0x09);
    _i2c->endTransmission();
}

uint8_t tvp5151::read_register(uint8_t address)
{
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(address);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_i2c_addr, (uint8_t)1);

    uint8_t reg_val = _i2c->read();
    return reg_val;
}