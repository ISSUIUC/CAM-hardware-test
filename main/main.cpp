#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define CORE_0 0
#define CORE_1 1


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

// Threads go here!
// Note: If you need to add a new thread, follow the example below & ALSO create an entry in init_tasks for the thread

void task_ex(void* arg) {
    while(true) {
        Serial.println("Example task running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Below is setup

[[noreturn]] void init_tasks() {
    xTaskCreatePinnedToCore(task_ex, "example", 1024, nullptr, 0, nullptr, CORE_0);
}

void setup()
{
    USB.begin();
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

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



    Serial.println("init");
    init_tasks();
}


void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}