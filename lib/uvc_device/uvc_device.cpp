#include "uvc_device.h"


extern SemaphoreHandle_t Sframe_rdy;
extern uint8_t *rx_frame_buf;
extern size_t received_frame_size;
extern esp_cam_ctlr_handle_t cam_handle; 
extern LCD_CAM_Module cam_ctrl;
extern tvp5151 tvp; 

camera cam1(CAM1_ON_OFF);

// I don't know how this is calculated but it was in the C file
#define UVC_MAX_FRAMESIZE_SIZE (60 * 1024)

static uvc_fb_t s_fb;

// GOOD reference:
//  https://github.com/espressif/esp-iot-solution/blob/master/examples/usb/device/usb_webcam/main/usb_webcam_main.c#L259
// https://github.com/espressif/esp-iot-solution/blob/aa0f3cbe0ce23babddb9202047119dc6b79b403c/docs/en/usb/usb_overview/tinyusb_development_guide.rst#L111

// when the uvc device is opened
static esp_err_t uvc_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
    (void)cb_ctx; // From what I've research it's just saying it's an unused parameter. 

    Serial.printf("UVC Start: format=%d, %dx%d @ %d fps\n", format, width, height, rate);

    // Initialize CAM_Controller (sets configs, sets call backs) // must enable before powering on camera as it causes a hard reset. (thought: memory fault due to pins being wiggled < kacper: " What does this mean?")

    cam_ctrl.enable_lcd_cam_controller();

    // Start CAM1

    cam1.enable_cam_power();

    // Wait for TVP5151 to lock onto video 

    tvp.tvp_wait_for_lock();



    return ESP_OK;
}

// when the uvc device wants a frame
static uvc_fb_t *uvc_get_fb_cb(void *cb_ctx)
{
    (void)cb_ctx;
    uint64_t us = (uint64_t)esp_timer_get_time();

    // implement this on another thread ig but idk if the variable will carry over.
    if (xSemaphoreTake(Sframe_rdy, pdMS_TO_TICKS(1000)))
    {
        s_fb.buf = rx_frame_buf;
        s_fb.len = received_frame_size;
        s_fb.width = FRAMESIZE_WIDTH;
        s_fb.height = FRAMESIZE_HEIGHT;
        s_fb.format = UVC_FORMAT_H264;
        s_fb.timestamp.tv_sec = us / 1000000UL;
        s_fb.timestamp.tv_usec = us % 1000000UL;
        return &s_fb;
    }
    else
    {
        Serial.println("[ERROR] [uvc_get_fb_cb] UVC FB CALL BACK - Waited 1 sec and no frame :(");
    }
    return nullptr;
}

static uvc_fb_t *uvc_get_fb_cb_with_encoding(void *cb_ctx)
{
    (void)cb_ctx;
    uint64_t us = (uint64_t)esp_timer_get_time();

    if (xSemaphoreTake(Sframe_rdy, pdMS_TO_TICKS(1000)))
    {
        // Encode frame to H264
        H264_ENC enc;
        esp_h264_enc_cfg_t enc_cfg = enc.set_config_H264_enc_single(ESP_H264_RAW_FMT_O_UYY_E_VYY, CAMERA_FPS, FRAMESIZE_HEIGHT, FRAMESIZE_WIDTH, BITRATE, QMIN, QMAX, GOP);
        esp_h264_err_t ret = enc.init_H264_enc_single(enc_cfg, HW);

        if (ret == ESP_H264_ERR_OK)
        {
            esp_h264_enc_in_frame_t *in_frame = enc.get_inframe();
            in_frame->raw_data.buffer = rx_frame_buf;
            in_frame->raw_data.len = received_frame_size;
            in_frame->pts = (uint32_t)millis();

            if (enc.run_H264_enc_single() == ESP_H264_ERR_OK)
            {
                esp_h264_enc_out_frame_t *out = enc.get_outframe();

                s_fb.buf = out->raw_data.buffer;
                s_fb.len = out->length;
                s_fb.width = FRAMESIZE_WIDTH;
                s_fb.height = FRAMESIZE_HEIGHT;
                s_fb.format = UVC_FORMAT_H264;
                s_fb.timestamp.tv_sec = us / 1000000UL;
                s_fb.timestamp.tv_usec = us % 1000000UL;
            }
            enc.close_H264_enc_single();
        }
        return &s_fb;
    }
    else
    {
        Serial.println("[ERROR] [uvc_get_fb_cb_with_encoding] UVC FB CALL BACK - Waited 1 sec and no frame :(");
    }

    return nullptr;
}

//  when uvc is done with the frame
static void uvc_return_fb_cb(uvc_fb_t *fb, void *cb_ctx)
{
    (void)cb_ctx;
    assert(fb == &s_fb);
}

// when host closes uvc device
static void uvc_stop_cb(void *cb_ctx)
{
    (void)cb_ctx;
    Serial.println("UVC Stopped");
    // later can implement encoder/other stopping logic here;
}

void UVC_device::init()
{
    // Allocate UVC buffer (should be larger than max frame size)
    // right now we use malloc might want to switch to heap_caps_malloc later...
    // max possible size

    uvc_buf = (uint8_t *)malloc(UVC_MAX_FRAMESIZE_SIZE);
    if (!uvc_buf)
    {
        Serial.println("Failed to allocate UVC buffer");
        while (1)
            ;
    }

    // Configure UVC device
    uvc_device_config_t uvc_config = {
        .uvc_buffer = uvc_buf,
        .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
        .start_cb = uvc_start_cb,
        .fb_get_cb = uvc_get_fb_cb_with_encoding,
        .fb_return_cb = uvc_return_fb_cb,
        .stop_cb = uvc_stop_cb,
        .cb_ctx = NULL,
    };

    esp_err_t ret = uvc_device_config(0, &uvc_config);
    if (ret != ESP_OK)
    {
        Serial.printf("UVC config failed: 0x%x\n", ret);
        while (1)
            ;
    }

    ret = uvc_device_init();
    if (ret != ESP_OK)
    {
        Serial.printf("UVC init failed: 0x%x\n", ret);
        while (1)
            ;
    }

    Serial.println("UVC Device initialized!");
}

void UVC_device::deinit()
{
    uvc_device_deinit();
    free(uvc_buf);
}
