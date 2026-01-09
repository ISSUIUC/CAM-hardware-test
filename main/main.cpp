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
#include <lcd_cam.h>            // kacper's code used for setting GPIO Matrix, can test initialize CAM Controller if esp_cam_ctlr doesn't work
#include <esp_cam_ctlr.h>       // esp code for cam controller
#include "esp_cam_ctlr_types.h" // for defining transaction type
#include "esp_cam_ctlr_dvp.h"

#include "esp_h264_enc_single.h"
#include "venc.h"

#include "esp_heap_caps.h"

#define CORE_0 0
#define CORE_1 1

// #define C_ENABLE_BUZZER

// #define C_ENABLE_TVP_DECODE

// #define CAM2_Select

// Video Test Pipeline #1 || TVP5151 Setup:

// TVP5151 Setup + Camera Init

// #define C_ENABLE_CAM_CONTROL
// #define CAM1_Select
// #define C_ENABLE_TVP_DECODE

// Video Test Pipeline #2 || TVP5151 Setup + CAM_Controller Initization + Output YUV422 || One FRAME

// TVP5151 Setup + Camera Init

// #define C_ENABLE_CAM_CONTROL
// #define CAM1_Select
// #define C_ENABLE_TVP_DECODE

// Set-up GPIO Matrix of ESP32 P4 || Initialize CAM Controller using esp driver

#define C_ENABLE_LCD_CAM_CONTROLLER

// Video Test Pipeline #3
// TVP5151 Setup + CAM_Controller Initization + Format Conversion YUV422 > YUV420 > USB-C

// TVP5151 Setup + Camera Init

// #define C_ENABLE_CAM_CONTROL
// #define CAM1_Select
// #define C_ENABLE_TVP_DECODE

// Set-up GPIO Matrix of ESP32 P4 || Initialize CAM Controller using esp driver

// #define C_ENABLE_LCD_CAM_CONTROLLER

// Convert Video Format from YUV422 to YUV420

// #define VIDEO_CONVERSION_YUV

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

static bool sent_once = false;

void on_frame_ready(const esp_cam_ctlr_trans_t *trans)
{
    if (sent_once)
    { // so that the function only sends one frame
        return;
    }
    sent_once = true;

    uint32_t off = 0;
    uint32_t magic = 0x314D4143;                   // "CAM1"
    uint32_t len = (uint32_t)trans->received_size; // length
    uint8_t *buf = (uint8_t *)trans->buffer;

    Serial.print("Setup Variables");

    Serial.print("Printing Magic");

    Serial.write((uint8_t *)&magic, 4);

    Serial.println("PRINTED Magic");

    Serial.println("Printing Length");

    Serial.print(len);
    Serial.println("PRINTED Length");

    while (off < len)
    { // making sure we don't overload usb write and instead send in chunks
        uint32_t chunk = len - off;

        if (chunk > 512)
            chunk = 512;

        Serial.println("Writing Chunk");
        size_t wrote = Serial.write(buf + off, chunk);
        Serial.println("PRINTED Chunk");

        off += chunk;

        if (wrote == 0)
        { // if nothing was written wait for USB to finish.
            Serial.println("Nothing was written, USB FULL");
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void setup()
{
    USB.begin();
    Serial.begin(115200);

    delay(15000);

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

    if (!tvp.set_output_format(VideoOutputFormat::DISCRETE_SYNC_YCBCR_422))
    { // enables 8-bit 4:2:2 YCbCr with discrete sync output
        Serial.println("TVP failed to enable output format 4:2:2");
        while (1)
        {
        };
    }
    Serial.println("TVP enable YCbCr Output Format Setup Success.");

    // Test all read functions
    Serial.println("\n=== TVP5151 Read Functions Test ===\n");

    // Test device ID
    uint16_t device_id = tvp.read_device_id();
    Serial.print("Device ID: 0x");
    Serial.println(device_id, HEX);

    // // Test gain factors
    // uint8_t cb_gain = tvp.read_cb_gain();
    // Serial.print("Cb Gain: 0x");
    // Serial.println(cb_gain, HEX);

    // uint8_t cr_gain = tvp.read_cr_gain();
    // Serial.print("Cr Gain: 0x");
    // Serial.println(cr_gain, HEX);

    // // Test lock status functions
    // Serial.println("\n--- Lock Status ---");

    // bool vsync_locked = tvp.read_vertical_sync_lock_status();
    // Serial.print("Vertical Sync Locked: ");
    // Serial.println(vsync_locked ? "YES" : "NO");

    // bool hsync_locked = tvp.read_horizontal_sync_lock_status();
    // Serial.print("Horizontal Sync Locked: ");
    // Serial.println(hsync_locked ? "YES" : "NO");

    // bool color_locked = tvp.read_color_subcarrier_lock_status();
    // Serial.print("Color Subcarrier Locked: ");
    // Serial.println(color_locked ? "YES" : "NO");

    // // Test other status functions
    // Serial.println("\n--- Other Status ---");

    // bool peak_white = tvp.read_peak_white_detect_status();
    // Serial.print("Peak White Detected: ");
    // Serial.println(peak_white ? "YES" : "NO");

    // bool vcr_mode = tvp.read_vcr_mode();
    // Serial.print("VCR Mode: ");
    // Serial.println(vcr_mode ? "YES" : "NO");

    // bool lost_lock = tvp.read_lost_lock_status();
    // Serial.print("Lost Lock Detected: ");
    // Serial.println(lost_lock ? "YES" : "NO");

    // bool lock_interrupt = tvp.read_lock_state_interrupt();
    // Serial.print("Lock State Interrupt: ");
    // Serial.println(lock_interrupt ? "YES" : "NO");

    // Serial.println("\n=== Test Complete ===\n");

    // bool weak_signal = tvp.read_weak_signal();
    // Serial.print("Weak Signal Detection: ");
    // Serial.println(weak_signal ? "Weak Signal Mode" : "No Weak Signal");

    // // Test all write functions
    // Serial.println("\n=== TVP5151 Write Functions Test ===\n");

    // // Test clock output enable
    // Serial.println("Testing set_clock_output_enable(true)...");
    // if (tvp.set_clock_output_enable(true))
    // {
    //     Serial.println("✓ Clock output enabled successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to enable clock output");
    // }
    // delay(100);

    // // Test YCbCr output enable
    // Serial.println("Testing set_ycbcr_output_enable(true)...");
    // if (tvp.set_ycbcr_output_enable(true))
    // {
    //     Serial.println("✓ YCbCr output enabled successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to enable YCbCr output");
    // }
    // delay(100);

    // // Test GPCL logic level
    // Serial.println("Testing set_gpcl_logic_level(true)...");
    // if (tvp.set_gpcl_logic_level(true))
    // {
    //     Serial.println("✓ GPCL logic level set to 1 successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to set GPCL logic level");
    // }
    // delay(100);

    // Test GPCL output
    // Serial.println("Testing set_gpcl_output(true)...");
    // if (tvp.set_gpcl_logic_level(true))
    // {
    //     Serial.println("✓ GPCL logic level is now high");
    // }
    // else
    // {
    //     Serial.println("✗ Failed");
    // }
    // delay(100);

    // Serial.println("Testing set_gpcl_or_vblk_output(false)... (meaning vblk output true)");
    // if (tvp.set_gpcl_or_vblk_output(false))
    // {
    //     Serial.println("✓ Vblk is now outputting");
    // }
    // else
    // {
    //     Serial.println("✗ failed");
    // }
    // delay(100);

    // Serial.println("Testing set_gpcl_output(true)...");
    // if (tvp.set_gpcl_or_vblk_output(true))
    // {
    //     Serial.println("✓ GPCL output configured successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to configure GPCL output");
    // }
    // delay(100);

    // Serial.println("Testing set_crop_avid_horizontal(15,-15)...");
    // if (tvp.set_crop_avid_horizontal(15, -15))
    // {
    //     Serial.println("✓ Hori crop set up successfully");
    // }
    // else
    // {
    //     Serial.println("✗ HORI CROP FAILED ...  :( ");
    // }
    // delay(100);

    // Serial.println("Testing set_crop_vblk_vertical(15,-15)...");
    // if (tvp.set_crop_vblk_vertical(15, -15))
    // {
    //     Serial.println("✓ vertical crop set up successfully!");
    // }
    // else
    // {
    //     Serial.println("✗ verti crop set up failed .. wamp wamp");
    // }
    // delay(100);

    // Serial.println("\nResetting crop registers...");
    // if (tvp.reset_crop())
    // {
    //     Serial.println("✓ Registers reset successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to reset registers");
    // }

    // // Reset miscellaneous controls register
    // Serial.println("\nResetting miscellaneous controls register...");
    // if (tvp.reset_miscellaneous_controls_register())
    // {
    //     Serial.println("✓ Register reset successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to reset register");
    // }

    // Serial.println("\n=== Write Functions Test Complete ===\n");

#endif

    Serial.println("Transition");

#ifdef C_ENABLE_LCD_CAM_CONTROLLER

    // test read register value Address: 0x0018
    // expected output 0111110001111111100000000011011 or 3E3FC01B in hex

    Serial.println("ENABLE LCD CAM Controller");

    // Serial.println(cam_ctrl.read_register_test(0x0018), HEX); dosen't work CRASHES.

    // Set up GPIO Matrix...

    Serial.println("Configure GPIO Matrix");

    cam_ctrl.cam_controller_configure_gpio_matrix(); // configure AVID, YOUT, HSYNC, VSYNC, PCLK pins // WORKS

    Serial.println("Configured");

    esp_cam_ctlr_handle_t cam_handle = NULL; // initialize a handle which esp uses to store data in

    // initialize LCD_CAM_Controller

    // cam_ctrl.initialize_cam_ctrl(); // Can test Kacper's initialize if esp dosen't work

    esp_cam_ctlr_dvp_pin_config pin_cfg{
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            GPIO_NUM_23,
            GPIO_NUM_22,
            GPIO_NUM_21,
            GPIO_NUM_20,
            GPIO_NUM_13,
            GPIO_NUM_12,
            GPIO_NUM_11,
            GPIO_NUM_10,
        },
        .vsync_io = GPIO_NUM_46,
        .de_io = GPIO_NUM_45,
        .pclk_io = GPIO_NUM_2,
        .xclk_io = GPIO_NUM_NC,
    };

    esp_cam_ctlr_dvp_config_t dvp_config;
    dvp_config.ctlr_id = 0;
    dvp_config.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_config.h_res = 720;
    dvp_config.v_res = 480;
    dvp_config.input_data_color_type = CAM_CTLR_COLOR_YUV422;
    dvp_config.cam_data_width = 8;

    dvp_config.bit_swap_en = 0;
    dvp_config.byte_swap_en = 0;
    dvp_config.bk_buffer_dis = 1;   /*!< Disable backup buffer */
    dvp_config.pin_dont_init = 1;   /*!< Don't initialize DVP pins if users have called "esp_cam_ctlr_dvp_init" before */
    dvp_config.pic_format_jpeg = 0; /*!< Input picture format is JPEG, if set this flag and "input_data_color_type" will be ignored */
    dvp_config.external_xtal = 1;

    dvp_config.dma_burst_size = 128;
    dvp_config.xclk_freq = 1;
    dvp_config.pin = &pin_cfg;

    // esp_cam_ctlr_dvp_config_t dvp_config = {
    // .ctlr_id = 0,
    // .clk_src = CAM_CLK_SRC_DEFAULT,
    // .h_res = 720,
    // .v_res = 480,
    // .input_data_color_type = CAM_CTLR_COLOR_YUV422,
    // .cam_data_width = 8,

    // .bit_swap_en = 0,               /*!< Enable bit swap */
    // .byte_swap_en = 0,              /*!< Enable byte swap
    //                                                 *
    //                                                 * GDMA Data Byte Order Table (input: B0,B1,B2,B3,B4,B5, addresses from low to high)
    //                                                 *
    //                                                 * | cam_data_width | bit_swap_en | byte_swap_en | Stage 1 Output Data Sequence   |
    //                                                 * |----------------|-------------|--------------|------------------------------  |
    //                                                 * | 8-bit          | 0           | 0            | {B0}{B1}{B2}{B3}{B4}{B5}       |
    //                                                 * | 8-bit          | 0           | 1            | {B1,B0}{B3,B2}{B5,B4}          |
    //                                                 * | 8-bit          | 1           | 0            | {B0'}{B1'}{B2'}{B3'}{B4'}{B5'} |
    //                                                 * | 8-bit          | 1           | 1            | {B1',B0'}{B3',B2'}{B5',B4'}    |
    //                                                 *
    //                                                 * | 16-bit         | 0           | 0            | {B1,B0}{B3,B2}{B5,B4}          |
    //                                                 * | 16-bit         | 0           | 1            | {B0,B1}{B2,B3}{B4,B5}          |
    //                                                 * | 16-bit         | 1           | 0            | {B1',B0'}{B3',B2'}{B5',B4'}    |
    //                                                 * | 16-bit         | 1           | 1            | {B0',B1'}{B2',B3'}{B4',B5'}    |
    //                                                 *
    //                                                 * | 24-bit         | 0           | 0            | {B2,B1,B0}{B5,B4,B3}           |
    //                                                 * | 24-bit         | 0           | 1            | {B0,B1,B2}{B3,B4,B5}           |
    //                                                 * | 24-bit         | 1           | 0            | {B2',B1',B0'}{B5',B4',B3'}     |
    //                                                 * | 24-bit         | 1           | 1            | {B0',B1',B2'}{B3',B4',B5'}     |
    //                                                 *
    //                                                 * Where B0' = bit-reversed B0，Bn'[7:0] = Bn[0:7]
    //                                                 * Each {} contains big-endian parallel data, {} are in serial relationship, output order is left to right
    //                                                 */
    // .bk_buffer_dis =1,            /*!< Disable backup buffer */
    // .pin_dont_init = 1,             /*!< Don't initialize DVP pins if users have called "esp_cam_ctlr_dvp_init" before */
    // .pic_format_jpeg = 0,           /*!< Input picture format is JPEG, if set this flag and "input_data_color_type" will be ignored */
    // .external_xtal = 1,

    // .dma_burst_size = 128,
    // .xclk_freq = 1,
    // .pin = &pin_cfg,
    // };

    Serial.println("Configure DVP Ctlr");

    esp_err_t err;
    err = esp_cam_new_dvp_ctlr(&dvp_config, &cam_handle); // WORKS No ERROR

    if (err != ESP_OK)
    {
        Serial.print("ERROR 1");
        Serial.println(err);
    }

#ifdef VIDEO_CONVERSION_YUV

    const cam_ctlr_format_conv_config_t conv_cfg = {
        .src_format = CAM_CTLR_COLOR_YUV422, // Source format: YUV422
        .dst_format = CAM_CTLR_COLOR_YUV420, // Destination format: YUV420
        .conv_std = COLOR_CONV_STD_RGB_YUV_BT601,
        .data_width = 8,
        .input_range = COLOR_RANGE_LIMIT,
        .output_range = COLOR_RANGE_LIMIT,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_format_conversion(cam_handle, &conv_cfg));

#endif

    Serial.println((uint32_t)cam_handle);

    Serial.println("Enable CAM Ctlr");

    esp_err_t err2;
    err2 = esp_cam_ctlr_enable(cam_handle); // enable high peripheral // no work

    if (err2 != ESP_OK)
    {
        Serial.print("ERROR 2");
        Serial.println(err2);
    }

    Serial.println("Start CAM Controller");

    esp_err_t err3;
    err3 = esp_cam_ctlr_start(cam_handle); // start cam // no work // might get through enable not do anything and go into start and fail.

    if (err3 != ESP_OK)
    {
        Serial.print("ERROR 3");
        Serial.println(err3);
    }

    Serial.print("CAM CONTROLLER START!");

    size_t frame_bytes = 720 * 480 * 2;

    esp_cam_ctlr_trans_t my_trans;

    // Allocate DMA/cache-aligned internal RAM for frame receive buffer, alignment is 64 bytes
    my_trans.buffer = heap_caps_aligned_alloc(64, frame_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!my_trans.buffer)
    {
        Serial.println("ERROR: failed to allocate aligned frame buffer");
    }

    my_trans.buflen = frame_bytes;

    Serial.println("Receive Frame");

    esp_err_t err4;
    err4 = esp_cam_ctlr_receive(cam_handle, &my_trans, ESP_CAM_CTLR_MAX_DELAY);
    if (err4 != ESP_OK)
    {
        Serial.print("ERROR 4");
        Serial.println(err4);
    }

    Serial.println("Received Frame");

    Serial.println("Running H264 encode->decode test");

    H264_ENC enc;
    esp_h264_enc_cfg_t enc_cfg = enc.set_config_H264_enc_single(ESP_H264_RAW_FMT_O_UYY_E_VYY, 24, 480, 720, 82944, 26, 30, 30);
    esp_h264_err_t ret = enc.init_H264_enc_single(enc_cfg, HW);
    if (ret != ESP_H264_ERR_OK)
    {
        Serial.println("ENC hardware init failed...trying software");
        ret = enc.init_H264_enc_single(enc_cfg, SW);
        if (ret != ESP_H264_ERR_OK)
            Serial.println("software failed too...");
    }
    else
    {

        esp_h264_enc_in_frame_t *in_frame = enc.get_inframe();
        esp_h264_pkt_t packet;
        packet.buffer = (uint8_t *)my_trans.buffer;
        packet.len = my_trans.buflen;
        in_frame->raw_data = packet;
        in_frame->pts = (uint32_t)millis(); // TODO: idk how to set pts/if this is the right way... (might wanna look at esp_h264_enc_dual.cpp&.h)
        esp_h264_err_t ret = enc.run_H264_enc_single();

        if (ret != ESP_H264_ERR_OK)
        {
            Serial.println("ENC process failed");
        }
        else
        {
            esp_h264_enc_out_frame_t *e_out_frame = enc.get_outframe();
            uint32_t enc_pkt_len = e_out_frame->length;
            uint8_t *enc_pkt = e_out_frame->raw_data.buffer;
            uint32_t enc_dts = e_out_frame->dts; // need to look into this plz
            uint32_t enc_pts = e_out_frame->pts; // need to look into this plz/i believe i can do this

            Serial.print("Encoded bytes: ");
            Serial.println((uint32_t)enc_pkt_len);

            Serial.println("Decoder time! Oh boi.");
            H264_DEC dec;
            esp_h264_enc_cfg_t dec_cfg = dec.set_config_H264_dec_single(ESP_H264_RAW_FMT_I420, 24, 480, 720, 82944, 26, 30, 30);
            if (dec.init_H264_dec_single(dec_cfg) != ESP_H264_ERR_OK)
            {
                Serial.println("DEC init failed");
            }
            else
            {
                esp_h264_dec_in_frame_t *in_frame = dec.get_inframe();
                // use the true length not the buffer's length might be a bad choice but we can go with it.
                in_frame->raw_data.buffer = enc_pkt;
                in_frame->raw_data.len = enc_pkt_len;

                // other option that we can use:
                // in_frame->raw_data = e_out_frame->raw_data;

                in_frame->dts = enc_dts;
                in_frame->pts = enc_pts;

                Serial.println("Decoding while loop....");
                while (in_frame->raw_data.len)
                {
                    esp_h264_err_t ret = dec.run_H264_dec_single();
                    if (ret != ESP_H264_ERR_OK)
                    {
                        Serial.print("Error code: ");
                        // ESP_H264_ERR_OK             = 0,   /*<! Succeeded */
                        // ESP_H264_ERR_FAIL = -1,            /*<! Failed */
                        // ESP_H264_ERR_ARG = -2,         /*<! Invalid arguments */
                        // ESP_H264_ERR_MEM = -3,         /*<! Insufficient memory */
                        // ESP_H264_ERR_UNSUPPORTED = -5, /*<! Un-supported */
                        // ESP_H264_ERR_TIMEOUT = -6,     /*<! Timeout */
                        // ESP_H264_ERR_OVERFLOW = -7,    /*<! Buffer overflow */
                        Serial.println(ret);
                        break;
                    }
                    in_frame->raw_data.buffer += in_frame->consume;
                    in_frame->raw_data.len -= in_frame->consume; // consume set by decoder as decoding happens
                }

                esp_h264_dec_out_frame_t *out_frame = dec.get_outframe();
                uint32_t out_yuv_len = out_frame->out_size;

                Serial.print("Decoded bytes: ");
                Serial.println((uint32_t)out_yuv_len);

                dec.close_H264_dec_single();
            }
            enc.close_H264_enc_single();
        }
    }

    Serial.println("Printing FRAME over serial");

    on_frame_ready(&my_trans); // send frame over usb.

    Serial.println("Printed");

#endif

    Serial.println("init");
    // init_tasks();
}

void loop()
{
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // // Test all read functions
    // Serial.println("\n=== TVP5151 Read Functions Test ===\n");

    // // Test device ID
    // uint16_t device_id = tvp.read_device_id();
    // Serial.print("Device ID: 0x");
    // Serial.println(device_id, HEX);

    // // Test gain factors
    // uint8_t cb_gain = tvp.read_cb_gain();
    // Serial.print("Cb Gain: 0x");
    // Serial.println(cb_gain, HEX);

    // uint8_t cr_gain = tvp.read_cr_gain();
    // Serial.print("Cr Gain: 0x");
    // Serial.println(cr_gain, HEX);

    // // Test lock status functions
    // Serial.println("\n--- Lock Status ---");

    // bool vsync_locked = tvp.read_vertical_sync_lock_status();
    // Serial.print("Vertical Sync Locked: ");
    // Serial.println(vsync_locked ? "YES" : "NO");

    // bool hsync_locked = tvp.read_horizontal_sync_lock_status();
    // Serial.print("Horizontal Sync Locked: ");
    // Serial.println(hsync_locked ? "YES" : "NO");

    // bool color_locked = tvp.read_color_subcarrier_lock_status();
    // Serial.print("Color Subcarrier Locked: ");
    // Serial.println(color_locked ? "YES" : "NO");

    // // Test other status functions
    // Serial.println("\n--- Other Status ---");

    // bool peak_white = tvp.read_peak_white_detect_status();
    // Serial.print("Peak White Detected: ");
    // Serial.println(peak_white ? "YES" : "NO");

    // bool vcr_mode = tvp.read_vcr_mode();
    // Serial.print("VCR Mode: ");
    // Serial.println(vcr_mode ? "YES" : "NO");

    // bool lost_lock = tvp.read_lost_lock_status();
    // Serial.print("Lost Lock Detected: ");
    // Serial.println(lost_lock ? "YES" : "NO");

    // bool lock_interrupt = tvp.read_lock_state_interrupt();
    // Serial.print("Lock State Interrupt: ");
    // Serial.println(lock_interrupt ? "YES" : "NO");

    // Serial.println("\n=== Test Complete ===\n");

    // bool weak_signal = tvp.read_weak_signal();
    // Serial.print("Weak Signal Detection: ");
    // Serial.println(weak_signal ? "Weak Signal Mode" : "No Weak Signal");

    // // Test all write functions
    // Serial.println("\n=== TVP5151 Write Functions Test ===\n");

    // // Test clock output enable
    // Serial.println("Testing set_clock_output_enable(true)...");
    // if (tvp.set_clock_output_enable(true))
    // {
    //     Serial.println("✓ Clock output enabled successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to enable clock output");
    // }
    // delay(100);

    // // Test YCbCr output enable
    // Serial.println("Testing set_ycbcr_output_enable(true)...");
    // if (tvp.set_ycbcr_output_enable(true))
    // {
    //     Serial.println("✓ YCbCr output enabled successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to enable YCbCr output");
    // }
    // delay(100);

    // // Test GPCL logic level
    // Serial.println("Testing set_gpcl_logic_level(true)...");
    // if (tvp.set_gpcl_logic_level(true))
    // {
    //     Serial.println("✓ GPCL logic level set to 1 successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to set GPCL logic level");
    // }
    // delay(100);

    // // Test GPCL output
    // Serial.println("Testing set_gpcl_output(true)...");
    // if (tvp.set_gpcl_or_vblk_output(true))
    // {
    //     Serial.println("✓ GPCL output configured successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to configure GPCL output");
    // }
    // delay(100);

    // Serial.println("Testing set_gpcl_output(true)...");
    // if (tvp.set_gpcl_or_vblk_output(true))
    // {
    //     Serial.println("✓ GPCL output configured successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to configure GPCL output");
    // }
    // delay(100);

    // Serial.println("Testing set_crop_avid_horizontal(15,-15)...");
    // if (tvp.set_crop_avid_horizontal(15, -15))
    // {
    //     Serial.println("✓ Hori crop set up successfully");
    // }
    // else
    // {
    //     Serial.println("✗ HORI CROP FAILED ...  :( ");
    // }
    // delay(100);

    // Serial.println("Testing set_crop_vblk_vertical(15,-15)...");
    // if (tvp.set_crop_vblk_vertical(15, -15))
    // {
    //     Serial.println("✓ vertical crop set up successfully!");
    // }
    // else
    // {
    //     Serial.println("✗ verti crop set up failed .. wamp wamp");
    // }
    // delay(100);

    // Serial.println("\nResetting crop registers...");
    // if (tvp.reset_crop())
    // {
    //     Serial.println("✓ Registers reset successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to reset registers");
    // }

    // // Reset miscellaneous controls register
    // Serial.println("\nResetting miscellaneous controls register...");
    // if (tvp.reset_miscellaneous_controls_register())
    // {
    //     Serial.println("✓ Register reset successfully");
    // }
    // else
    // {
    //     Serial.println("✗ Failed to reset register");
    // }

    // Serial.println("\n=== Write Functions Test Complete ===\n");
}