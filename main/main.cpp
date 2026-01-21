#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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
#include <lcd_cam.h>            // kacper's code used for setting GPIO Matrix, can test initialize CAM Controller if esp_cam_ctlr doesn't work
#include <esp_cam_ctlr.h>       // esp code for cam controller
#include "esp_cam_ctlr_types.h" // for defining transaction type
#include "esp_cam_ctlr_dvp.h"

#include "esp_h264_enc_single.h"
#include "venc.h"

#include "esp_heap_caps.h"
#include "uvc_device.h"

#define CORE_0 0
#define CORE_1 1

// #define C_ENABLE_BUZZER

// #define C_ENABLE_TVP_DECODE
//------------------------------------------------

#define C_ENABLE_CAM_CONTROL
#define CAM2_Select // RunCam is on AIP1B, not AIP1A
#define C_ENABLE_TVP_DECODE
#define UVC_USB_DEVICE

//---------------------------

// RHSoftwareSPI _rspi;
// RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);

tvp5151 tvp(TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire);
LCD_CAM_Module cam_ctrl;
UVC_device uvc;
esp_cam_ctlr_handle_t cam_handle = NULL;
SemaphoreHandle_t Sframe_rdy = NULL;



#ifdef IS_CAM
const int LED_PIN = 51;
#endif

#ifdef IS_EAGLE
const int LED_PIN = 22;
#endif


// Threads go here!
// Note: If you need to add a new thread, follow the example below & ALSO create an entry in init_tasks for the thread

void task_ex(void *arg)
{
    while (true)
    {
        Serial.println("Example task running...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Below is setup

[[noreturn]] void init_tasks()
{
    // xTaskCreatePinnedToCore(task_h264_encode_decode, "h264_test", 4096, nullptr, 1, nullptr, CORE_1);

    // Example task
    xTaskCreatePinnedToCore(task_ex, "example", 1024, nullptr, 0, nullptr, CORE_0);
}

void setup()
{
    USB.begin();
    Serial.begin(115200);

#ifdef IS_CAM
    Wire.begin(I2C_SDA, I2C_SCL);
#endif

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

    Serial.println("Cam dvp controller init");

    Sframe_rdy = xSemaphoreCreateBinary();
    if (!Sframe_rdy)
    {
        Serial.println("Failed to create frame semaphore");
        while (1)
        {
        };
    }

    Serial.println("Configure GPIO Matrix");

    cam_ctrl.cam_controller_configure_gpio_matrix(); // configure AVID, YOUT, HSYNC, VSYNC, PCLK pins // WORKS
   
    Serial.println("Configured");


#ifdef C_ENABLE_TVP_DECODE

    for (uint8_t i = 0; i < 8; i++)
    {
        pinMode(YOUT[i], INPUT);
    }
    Serial.println("Set up YOUT");

    if (!tvp.init())
    {
        Serial.println("TVP failed to init.");
        while (1)
        {
        };
    }
    Serial.println("TVP Success INIT");

#ifdef CAM1_Select
    if (!tvp.source_select(CAM1))
    {
        Serial.println("CAM 1 failed to select.");
        while (1)
        {
        };
    }
    Serial.println("CAM1 Success Select");
#endif

#ifdef CAM2_Select
    if (!tvp.source_select(CAM2))
    {
        Serial.println("CAM 2 failed to select.");
        while (1)
        {
        };
    }
    Serial.println("CAM2 Success Select");
#endif

    if (!tvp.set_ycbcr_output_enable(true))
    {
        Serial.println("TVP failed to enable output data.");
        while (1)
        {
        };
    }
    Serial.println("TVP Output Data Success.");

    if (!tvp.set_clock_output_enable(true))
    {
        Serial.println("TVP failed to enable sclk.");
        while (1)
        {
        };
    }
    Serial.println("TVP enable sclk Setup Success.");

    if (!tvp.set_avid_output_enable(true))
    {
        Serial.println("TVP failed to enable AVID output.");
        while (1)
        {
        };
    }
    Serial.println("TVP AVID Output Enabled.");

    if (!tvp.set_yCbCr_output_format(true))
    {
        Serial.println("TVP failed to enable output format 4:2:2");
        while (1)
        {
        };
    }
    Serial.println("TVP enable YCbCr Output Format Setup Success.");

    // Test device ID
    uint16_t device_id = tvp.read_device_id();
    Serial.print("Device ID: 0x");
    Serial.println(device_id, HEX);

    // Check vertical line count - if this shows ~525 for NTSC, video IS being processed
    uint16_t line_count = tvp.read_vertical_line_count();
    Serial.print("Vertical Line Count: ");
    Serial.println(line_count);

    bool vsync_locked = tvp.read_vertical_sync_lock_status();
    Serial.print("Vertical Sync Locked: ");
    Serial.println(vsync_locked ? "YES" : "NO");

    bool hsync_locked = tvp.read_horizontal_sync_lock_status();
    Serial.print("Horizontal Sync Locked: ");
    Serial.println(hsync_locked ? "YES" : "NO");

    bool color_locked = tvp.read_color_subcarrier_lock_status();
    Serial.print("Color Subcarrier Locked: ");
    Serial.println(color_locked ? "YES" : "NO");


    
    #ifdef UVC_USB_DEVICE
    uvc.init(); // this basically starts the uvc device and the video transfer on another thread so we don't need to worry about it in void loop
    #endif


#endif

    delay(1000);
}

void loop()
{

    vTaskDelay(pdMS_TO_TICKS(1000));
}










