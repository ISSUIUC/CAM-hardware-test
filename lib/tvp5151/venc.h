#include <Wire.h>
#include <Arduino.h>

#include "esp_h264_enc_single_hw.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_dual_hw.h"
#include "esp_h264_enc_dual.h"

class H264 {
    private:
    //int frame_buf1 [2][32];      //store in, out frame here instead of dynmaically allocating space every time? Need to configure size
    //int frame_buf2 [2][32];      //store in, out frame here instead of dynmaically allocating space every time? Need to configure size

    esp_h264_enc_cfg_hw_t cfg;
    esp_h264_enc_in_frame_t in_frame;
    esp_h264_enc_out_frame_t out_frame;
    esp_h264_enc_handle_t enc;

    public:
    esp_h264_enc_cfg_hw_t set_hw_H264_config_single(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop);

    //configures both using same settings
    //esp_h264_enc_cfg_dual_hw_t set_hw_H264_config_dual(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop);

    esp_h264_enc_in_frame_t* get_inframe();
    esp_h264_enc_out_frame_t* get_ouyframe();

    //adapted from esp_h264_hw_enc_test.c - need to modify
    //need if(ESP_H264_ERR_OK != init_hw_H264_enc_single()) close_hw_H264_enc_single();, same for run
    esp_h264_err_t init_hw_H264_enc_single(esp_h264_enc_cfg_hw_t config);
    esp_h264_err_t run_hw_H264_enc_single();
    esp_h264_err_t close_hw_H264_enc_single();
};