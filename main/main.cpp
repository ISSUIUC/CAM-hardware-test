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
#include <lcd_cam.h> // kacper's code used for setting GPIO Matrix, can test initialize CAM Controller if esp_cam_ctlr doesn't work 
#include <esp_cam_ctlr.h> // esp code for cam controller 
#include "esp_cam_ctlr_types.h" // for defining transaction type


#include "esp_h264_enc_single.h"

#define CORE_0 0
#define CORE_1 1

// #define C_ENABLE_BUZZER




#define C_ENABLE_TVP_DECODE

// #define CAM2_Select



// Video Test Pipeline #1 || TVP5151 Setup:

    // TVP5151 Setup + Camera Init

// #define C_ENABLE_CAM_CONTROL
// #define CAM1_Select
// #define C_ENABLE_TVP_DECODE


// Video Test Pipeline #2 || TVP5151 Setup + CAM_Controller Initization + Output YUV422 

    // TVP5151 Setup + Camera Init
// #define C_ENABLE_CAM_CONTROL
// #define CAM1_Select
// #define C_ENABLE_TVP_DECODE


    // Set-up GPIO Matrix of ESP32 P4 || Initialize CAM Controller using esp driver 

// #define C_ENABLE_LCD_CAM_CONTROLLER











// Video Test Pipeline #3 
    // TVP5151 Setup + CAM_Controller Initization + Format Conversion YUV422 > YUV420







// RHSoftwareSPI _rspi;
// RH_RF24 radio(SI4463_CS, SI4463_INT, SI4463_SDN);

tvp5151 tvp(TVP5151_PDN, TVP5151_RESET, TVP5151_ADDR, &Wire);
LCD_CAM_Module cam_ctrl;

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

    #ifdef CAM1_Select

    if (!tvp.source_select(CAM1))
    {
        Serial.println("CAM 1 failed to select.");
        while (1)
        {
        };
    }
    #endif

    #ifdef CAM2_Select

        if (!tvp.source_select(CAM2))
        {
            Serial.println("CAM 2 failed to select.");
            while (1)
            {
            };
        }
    #endif

    if(!tvp.set_ycbcr_output_enable(true)){
        Serial.println("TVP failed to enable output data.");
        while (1)
        {
        };
    }

    if(!tvp.set_clock_output_enable(true)){
         Serial.println("TVP failed to enable sclk.");
        while (1)
        {
        };
    }

     if(!tvp.set_yCbCr_output_format(true)){ // enables 8-bit 4:2:2 YCbCr with discrete sync output 
         Serial.println("TVP failed to enable output format 4:2:2");
        while (1)
        {
        };
    }


    // Test all read functions
    Serial.println("\n=== TVP5151 Read Functions Test ===\n");

    // Test device ID
    uint16_t device_id = tvp.read_device_id();
    Serial.print("Device ID: 0x");
    Serial.println(device_id, HEX);

    // Test gain factors
    uint8_t cb_gain = tvp.read_cb_gain();
    Serial.print("Cb Gain: 0x");
    Serial.println(cb_gain, HEX);

    uint8_t cr_gain = tvp.read_cr_gain();
    Serial.print("Cr Gain: 0x");
    Serial.println(cr_gain, HEX);

    // Test lock status functions
    Serial.println("\n--- Lock Status ---");

    bool vsync_locked = tvp.read_vertical_sync_lock_status();
    Serial.print("Vertical Sync Locked: ");
    Serial.println(vsync_locked ? "YES" : "NO");

    bool hsync_locked = tvp.read_horizontal_sync_lock_status();
    Serial.print("Horizontal Sync Locked: ");
    Serial.println(hsync_locked ? "YES" : "NO");

    bool color_locked = tvp.read_color_subcarrier_lock_status();
    Serial.print("Color Subcarrier Locked: ");
    Serial.println(color_locked ? "YES" : "NO");

    // Test other status functions
    Serial.println("\n--- Other Status ---");

    bool peak_white = tvp.read_peak_white_detect_status();
    Serial.print("Peak White Detected: ");
    Serial.println(peak_white ? "YES" : "NO");

    bool vcr_mode = tvp.read_vcr_mode();
    Serial.print("VCR Mode: ");
    Serial.println(vcr_mode ? "YES" : "NO");

    bool lost_lock = tvp.read_lost_lock_status();
    Serial.print("Lost Lock Detected: ");
    Serial.println(lost_lock ? "YES" : "NO");

    bool lock_interrupt = tvp.read_lock_state_interrupt();
    Serial.print("Lock State Interrupt: ");
    Serial.println(lock_interrupt ? "YES" : "NO");

    Serial.println("\n=== Test Complete ===\n");

    bool weak_signal = tvp.read_weak_signal();
    Serial.print("Weak Signal Detection: ");
    Serial.println(weak_signal ? "Weak Signal Mode" : "No Weak Signal");



    // Test all write functions
    Serial.println("\n=== TVP5151 Write Functions Test ===\n");

    // Test clock output enable
    Serial.println("Testing set_clock_output_enable(true)...");
    if (tvp.set_clock_output_enable(true))
    {
        Serial.println("✓ Clock output enabled successfully");
    }
    else
    {
        Serial.println("✗ Failed to enable clock output");
    }
    delay(100);

    // Test YCbCr output enable
    Serial.println("Testing set_ycbcr_output_enable(true)...");
    if (tvp.set_ycbcr_output_enable(true))
    {
        Serial.println("✓ YCbCr output enabled successfully");
    }
    else
    {
        Serial.println("✗ Failed to enable YCbCr output");
    }
    delay(100);

    // Test GPCL logic level
    Serial.println("Testing set_gpcl_logic_level(true)...");
    if (tvp.set_gpcl_logic_level(true))
    {
        Serial.println("✓ GPCL logic level set to 1 successfully");
    }
    else
    {
        Serial.println("✗ Failed to set GPCL logic level");
    }
    delay(100);

    // Test GPCL output
    Serial.println("Testing set_gpcl_output(true)...");
    if (tvp.set_gpcl_output(true))
    {
        Serial.println("✓ GPCL output configured successfully");
    }
    else
    {
        Serial.println("✗ Failed to configure GPCL output");
    }
    delay(100);

    // Reset miscellaneous controls register
    Serial.println("\nResetting miscellaneous controls register...");
    if (tvp.reset_miscellaneous_controls_register())
    {
        Serial.println("✓ Register reset successfully");
    }
    else
    {
        Serial.println("✗ Failed to reset register");
    }

    Serial.println("\n=== Write Functions Test Complete ===\n");
 




#endif



// #ifdef C_ENABLE_LCD_CAM_CONTROLLER

    // test read register value Address: 0x0018
    // expected output 0111110001111111100000000011011 or 3E3FC01B in hex

    Serial.println(cam_ctrl.read_register(0x0018));

    ESP_ERROR_CHECK(cam_ctrl.cam_controller_configure_gpio_matrix()); // configure AVID, YOUT, HSYNC, VSYNC, PCLK pins

    esp_cam_ctlr_handle_t cam_handle = NULL; // initialize a handle which esp uses to store data in 

    esp_cam_ctlr_trans_t my_trans;

    // initialize LCD_CAM_Controller 

    // cam_ctrl.initialize_cam_ctrl(); // Can test Kacper's initialize if esp dosen't work

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(&cam_handle)); // enable high peripheral

    ESP_ERROR_CHECK(esp_cam_ctlr_start(&cam_handle)); // start cam controller


    ESP_ERROR_CHECK(esp_cam_ctlr_receive(&cam_handle,&my_trans ,ESP_CAM_CTLR_MAX_DELAY)







    // Set up GPIO Matrix...







// #endif

    Serial.println("init");
    init_tasks();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}