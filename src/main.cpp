#include <Arduino.h>
#include <pins.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF24.h>
#include <RHSoftwareSPI.h>
#include <RHHardwareSPI.h>
#include <tvp5151.h>

// RHSoftwareSPI _rspi;
// RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);

tvp5151 tvp(TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire);

#ifdef IS_CAM
const int LED_PIN = 51;
#endif

#ifdef IS_EAGLE
const int LED_PIN = 22;
#endif

void setup()
{

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);

    // while (!Serial)
    // {
    // };
    // delay(10);
    Serial.println("init");
    tvp.init();
    // SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);


}

void loop()
{

#ifdef IS_CAM
    Serial.println("Requesting device ID from 0x5C...");
    Serial.print("Device ID response: 0x");
    Serial.println(tvp.read_device_id(), HEX);

    // Wire.beginTransmission(0x40);
    // Wire.write(0x05);
    // if(Wire.endTransmission(true)) {
    //     Serial.println(":(");
    //     return;
    // }

    // Wire.requestFrom(0x40, 2);
    // int a = Wire.read();
    // int b = Wire.read();
    // Serial.println("ok!");
    // Serial.println(((a<<8) + b) * 0.003125);

    // 3.125 mV/LSB

    // scan

    // Serial.println("SCAN: ----");
    // for(uint8_t i = 1; i < 0xFF; i++) {

    //     Wire.beginTransmission(i);
    //     uint8_t errn = Wire.endTransmission();

    //     if (errn == 0) {
    //         Serial.print("I2C DETECT: 0x");
    //         Serial.println(i, HEX);
    //     }

    //     delay(2);

    // }
    // Serial.println("END SCAN ----");





    // Serial.println("loop");
    delay(500);
#endif
}