#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"

USBCDC USBSerial;
#define Serial USBSerial

#include <pins.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF24.h>
#include <RHSoftwareSPI.h>
#include <RHHardwareSPI.h>
#include <tvp5151.h>

#include "esp_h264_enc_single.h"

// #define C_ENABLE_BUZZER
// #define C_ENABLE_CAM_CONTROL
// #define C_ENABLE_TVP_DECODE

// RHSoftwareSPI _rspi;
// RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);

// tvp5151 tvp(TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire);

#ifdef IS_CAM
const int LED_PIN = 51;
#endif

#ifdef IS_EAGLE
const int LED_PIN = 22;
#endif

void setup()
{
    USB.begin();
    Serial.begin(115200);

    // Wire.begin(I2C_SDA, I2C_SCL);

    #ifdef C_ENABLE_BUZZER
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);


        tone(BUZZER_PIN, 2700);
        delay(200);
        noTone(BUZZER_PIN);

    #endif


    while (!Serial)
    {
    };
    delay(50);
    
    #ifdef C_ENABLE_TVP_DECODE

    for(uint8_t i = 0; i < 8; i++) {
        pinMode(YOUT[i], INPUT);
    }
    Serial.println("Set up YOUT");

    if(!tvp.init()) {
        Serial.println("TVP failed to init.");
        // while (1) {};
        Serial.println("Continuing anyway...");
        delay(1000);
    }
    tvp._debug_set_reg();


    #endif

    #ifdef C_ENABLE_CAM_CONTROL
        pinMode(CAM1_ON_OFF, OUTPUT);
        pinMode(LED_RED, OUTPUT);
        // Serial1.begin(9600, CAM)
        digitalWrite(CAM1_ON_OFF, LOW);
        digitalWrite(LED_RED, LOW);

        Serial.println("WARNING : Turning on camera in 1.5s!");
        delay(1500);

        digitalWrite(CAM1_ON_OFF, HIGH);
        digitalWrite(LED_RED, HIGH);

    #endif


    
    // SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    Serial.println("init");


}

void loop()
{

#ifdef IS_CAM

    #ifdef C_ENABLE_TVP_DECODE
    for(int i = 0; i < 8; i++) {
        Serial.print(digitalRead(YOUT[i]) ? 1 : 0);
    }
    Serial.println();
    #endif

    // Serial.println("loop");
    delay(1);
#endif
}