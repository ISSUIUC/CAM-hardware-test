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



// for the registers should I have a boolean for each bit such as black output if I want it on or should this just select camera in which it's decoding? I think the latter
void tvp5151::TVP_CAM_Decoder_Select(CAM_SELECT CAM) {
    
    if(CAM){ // CAM1 (AIP1B)
        write_register(TVP_INPUT_SOURCE_SELECTION,2);
    }
    else{ // CAM0 (AIP1A)
        write_register(TVP_INPUT_SOURCE_SELECTION,0);
    }
}

void tvp5151::_debug_set_misc_controls() {
    // Sets registers according to the default spec 1
    write_register(TVP_REG_MISC_CONTROL, 0x09);// 0x09 Enables clock & YCbCr output
}


// Outputs and Data Rates Select Register 
// Extended Range vs BT.601 Limited Range (We are using default (Extended code range) and if video dosen't look right it's said that this is a common error with video so could be worth checking into more)
// ESP32 / FPGA processing Reccomends extended range so this is what is set by default  

// 3.21.16 Active Video Cropping Start Pixel MSB Register
// We are not doing any sort of AVID right? Because our CAM Signal alligns with video decoder standard 720 x 480 ? 

// 3.21.20 Genlock and RTC Register
// Real Time Communcation I need to look into this more and if we need it as we are ultimately wanting to stream live video? Does it correlate at all? 



// 3.21.33 Cb Gain Factor Register
uint8_t tvp5151::read_cb_gain() { 
    tvp_i2c_result_t reg_val = read_register(TVP_REF_CB_GAIN_FACTOR);
    if(reg_val.result == SUCCESS){
        return reg_val.data;
    }
    return -1;
}

uint8_t tvp5151::read_cr_gain() { 
    tvp_i2c_result_t reg_val = read_register(TVP_REF_CR_GAIN_FACTOR);
    if(reg_val.result == SUCCESS){
        return reg_val.data;
    }
    return -1;
}


// 3.21.40 MSB of Device ID Register
// 3.21.41 LSB of Device ID Register

uint16_t tvp5151::read_device_id() {

    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(TVP_DEVICE_ID_MSB);
    uint8_t error = _i2c->endTransmission(true);
    if(error != SUCCESS){
        print_I2C_error(categorize_error(error));
        return -1;
    }
    
    _i2c->requestFrom(_i2c_addr, 2);
    uint8_t id_lsb = _i2c->read();
    uint8_t id_msb = _i2c->read();
    uint16_t device_id = (id_msb << 8) + id_lsb;
    
    return -1;
}


// 3.21.44 Vertical Line Count MSB Register (the only useful bits are 1 and 0 so can I do the same thing of << 8 for MSB and then + LSB ?)
// 3.21.45 Vertical Line Count LSB Register (Used for seeing how many lines per frame)







tvp_i2c_result_t tvp5151::read_register(uint8_t register_addr){
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(register_addr);
    tvp_i2c_result_t reg_val;


    uint8_t error = _i2c->endTransmission(true);
    reg_val.result = categorize_error(error);

    if(error) {
        Serial.println("I2C read failed on read_register");
        print_I2C_error(reg_val.result);
        return reg_val;
    }

    _i2c->requestFrom(_i2c_addr, 1);
    uint8_t register_data = _i2c->read();
    reg_val.data = register_data;
    return reg_val;

}

void tvp5151::write_register(uint8_t register_addr, uint8_t data){
    _i2c->beginTransmission(_i2c_addr);
    _i2c->write(register_addr);
    _i2c->write(data);
    uint8_t error = _i2c->endTransmission(true);
    if(error) {
        Serial.println("I2C read failed on write_register");
        print_I2C_error(categorize_error(error));
    }
}


void tvp5151::print_I2C_error(TVP_I2C_ERROR error){
    switch(error) {
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

TVP_I2C_ERROR tvp5151::categorize_error(uint8_t error){
    switch(error) {
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