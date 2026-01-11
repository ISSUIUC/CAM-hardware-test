#include "uvc_device.h"

extern SemaphoreHandle_t Sframe_rdy;
extern uint8_t *rx_frame_buf;
extern size_t received_frame_size;

// I don't know how this is calculated but it was in the C file
#define UVC_MAX_FRAMESIZE_SIZE (60 * 1024)
#define WIDTH CONFIG_UVC_CAM1_FRAMESIZE_WIDTH
#define HEIGHT CONFIG_UVC_CAM1_FRAMESIZE_HEIGT

static uvc_fb_t s_fb;

// when the uvc device is opened
static esp_err_t uvc_start_cb(uvc_format_t format, int width, int height, int rate, void *cb_ctx)
{
    (void)cb_ctx; // don't really know what this does but it's in the example code

    Serial.printf("UVC Start: format=%d, %dx%d @ %d fps\n", format, width, height, rate);
    // can later implement starting functions here like starting encoder and camera :)
    return ESP_OK;
}

// when the uvc device wants a frame. (implement in main.cpp?)
static uvc_fb_t *uvc_get_fb_cb(void *cb_ctx)
{
    (void)cb_ctx;
    uint64_t us = (uint64_t)esp_timer_get_time();

    // implement this on another thread ig but idk if the variable will carry over.
    if (xSemaphoreTake(Sframe_rdy, pdMS_TO_TICKS(1000)))
    {
        s_fb.buf = rx_frame_buf;
        s_fb.len = received_frame_size;
        s_fb.width = WIDTH;
        s_fb.height = HEIGHT;
        s_fb.format = UVC_FORMAT_H264;
        s_fb.timestamp.tv_sec = us / 1000000UL;
        s_fb.timestamp.tv_usec = us % 1000000UL;
        return &s_fb;
    }
    else
    {
        Serial.println("Waited 1 sec and no frame :(");
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
    /*
       uint8_t *uvc_buffer = (uint8_t *)malloc(UVC_MAX_FRAMESIZE_SIZE);
    TEST_ASSERT_NOT_NULL(uvc_buffer);

    uvc_device_config_t config = {
        .uvc_buffer = uvc_buffer,
        .uvc_buffer_size = UVC_MAX_FRAMESIZE_SIZE,
        .start_cb = camera_start_cb,
        .fb_get_cb = camera_fb_get_cb,
        .fb_return_cb = camera_fb_return_cb,
        .stop_cb = camera_stop_cb,
        .cb_ctx = NULL,
    };
    uvc_device_config(0, &config);
    uvc_device_init();
    for (int i = 0; i < TEST_COUNT; i++)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "UVC Device Test: %d", i);
    }
    uvc_device_deinit();
    free(uvc_buffer);
    */

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
        .fb_get_cb = uvc_get_fb_cb,
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
