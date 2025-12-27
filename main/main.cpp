#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "USB.h"
#include "USBCDC.h"

USBCDC USBSerial;
#undef Serial
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

// #define WAIT_FOR_SERIAL

#ifdef IS_CAM
#define C_ENABLE_BUZZER
// #define C_ENABLE_CAM_CONTROL
// #define C_ENABLE_TVP_DECODE
#define C_ENABLE_TX
#endif

#ifdef IS_EAGLE
#define E_ENABLE_RX
#endif



// tvp5151 tvp(TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire);

#ifdef IS_CAM
const int LED_PIN = 51;
#endif

#ifdef IS_EAGLE
const int LED_PIN = 22;
#endif

// Threads go here!
// Note: If you need to add a new thread, follow the example below & ALSO create an entry in init_tasks for the thread

void task_radio_tx(void* arg) {
    RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);
    Serial.println("Initializing radio..");
    if(!radio.init()) {
        Serial.println("Radio failed to init");
        while(true) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    Serial.println("Init successfully");
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t dat[3] = {0xab, 0xcd, 0xef};
    bool r_st = false;

    while(true) {
        radio.send(dat, 3);
        if(radio.waitPacketSent(100)) {
            r_st = !r_st;
            digitalWrite(LED_GREEN, r_st);
            Serial.print(".");
        } else {
            Serial.print("x");
            digitalWrite(LED_GREEN, LOW);
        }
    }
}


  void task_radio_rx(void* arg) {
      RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);
      Serial.println("Initializing radio..");
      if(!radio.init()) {
          Serial.println("Radio failed to init");
          while(true) { vTaskDelay(pdMS_TO_TICKS(1)); }
      }
      Serial.println("Init successfully");

      uint8_t dat[10];
      uint8_t len = sizeof(dat);

      // Force into RX mode
      radio.available();

      while(true) {
          radio.handleInterrupt();

          if(radio.available()) {
              if(radio.recv(dat, &len)) {
                  Serial.println("Got packet!");
              }
          }

          // Debug: print modem/interrupt status every 2 seconds
          static uint32_t last_print = 0;
          if(millis() - last_print > 2000) {
              last_print = millis();

              uint8_t int_status[8];
              uint8_t clear_none[] = {0, 0, 0};  // Don't clear any interrupts
              radio.command(0x20, clear_none, 3, int_status, 8);  // GET_INT_STATUS

              Serial.print("INT_PEND="); Serial.print(int_status[0], HEX);
              Serial.print(" PH_PEND="); Serial.print(int_status[2], HEX);
              Serial.print(" MODEM_PEND="); Serial.print(int_status[4], HEX);
              Serial.print(" Mode="); Serial.println(radio.mode());
          }

          vTaskDelay(1);
      }
  }
// Below is setup

[[noreturn]] void init_tasks() {
    #ifdef C_ENABLE_TX
        xTaskCreatePinnedToCore(task_radio_tx, "radio", 8192, nullptr, 0, nullptr, CORE_0);
    #endif
    #ifdef E_ENABLE_RX
        xTaskCreatePinnedToCore(task_radio_rx, "radio", 8192, nullptr, 0, nullptr, CORE_0);
    #endif

    while(true) {
        delay(1000);
        // Serial.println("Running");
    }
}

void setup()
{
    USB.begin();
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);

    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_ORANGE, LOW);

    #ifdef C_ENABLE_BUZZER
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);


        tone(BUZZER_PIN, 2700);
        delay(200);
        noTone(BUZZER_PIN);

    #endif


    #ifdef WAIT_FOR_SERIAL
    while (!Serial)
    {
    };
    #endif
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