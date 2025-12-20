#include <tvp5151.h>

tvp5151::tvp5151(uint8_t pdn, uint8_t reset, uint8_t addr, TwoWire* i2c) {
    _i2c_addr = addr;
    _i2c = i2c;
    _pdn = pdn;
    _reset = reset;
}

bool tvp5151::init() {
    pinMode(_pdn, OUTPUT);
    pinMode(_reset, OUTPUT);

    digitalWrite(_pdn, HIGH);
    delay(20);

    digitalWrite(_reset, LOW);
    delayMicroseconds(500);
    digitalWrite(_reset, HIGH);
    delayMicroseconds(250);

    uint16_t dev_id = read_device_id();

    return dev_id == 0x5151;
}

void tvp5151::_debug_set_reg() {
    // Sets registers according to the default spec 1
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_REG_MISC_CONTROL);
    _i2c->write(0x09); //  Enable clock & YCbCr output
    uint8_t err = _i2c->endTransmission(true);

    if(err) {
        Serial.println("I2C read failed on debug_set_reg");
        Serial.print("ERRNO ");
        Serial.println(err);
    }

}

uint16_t tvp5151::read_device_id() {
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_DEVICE_ID_MSB);
    uint8_t err = _i2c->endTransmission(true);
    if(err) {
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