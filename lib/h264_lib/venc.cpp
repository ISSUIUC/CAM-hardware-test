#include "venc.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_h264_alloc.h"
#include "../../components/esp_h264/test_apps/main/h264_io.h"

esp_h264_enc_cfg_t H264_ENC::set_config_H264_enc_single(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop)
{
    esp_h264_enc_cfg_t config;

    config.pic_type = format;
    config.gop = gop;
    config.fps = fps;
    config.rc = (esp_h264_enc_rc_t){bitrate, qp_min, qp_max};
    config.res = (esp_h264_resolution_t){width, height};

    return config;
}

esp_h264_enc_in_frame_t *H264_ENC::get_inframe()
{
    return &in_frame;
}

esp_h264_enc_out_frame_t *H264_ENC::get_outframe()
{
    return &out_frame;
}

// adapted from esp_h264_hw_enc_test.c:19, change calloc function calls
esp_h264_err_t H264_ENC::init_H264_enc_single(esp_h264_enc_cfg_t config, enc_type type)
{
    cfg = config;
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    uint16_t width = ((cfg.res.width + 15) >> 4 << 4); // possibly remove (only in some of example code)
    uint16_t height = ((cfg.res.height + 15) >> 4 << 4);

    in_frame.raw_data.len = (int)((float)width * height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.pic_type));
    in_frame.raw_data.buffer = (uint8_t *)esp_h264_aligned_calloc(16, 1, in_frame.raw_data.len, &in_frame.raw_data.len, ESP_H264_MEM_INTERNAL);
    if (!in_frame.raw_data.buffer)
    {
        printf("mem allocation failed.line %d %d %d %d\n", width, height, (int)in_frame.raw_data.len, __LINE__);
        return ESP_H264_ERR_MEM;
    }

    out_frame.raw_data.len = in_frame.raw_data.len;
    out_frame.raw_data.buffer = (uint8_t *)esp_h264_aligned_calloc(16, 1, out_frame.raw_data.len, &out_frame.raw_data.len, ESP_H264_MEM_INTERNAL);
    if (!out_frame.raw_data.buffer)
    {
        printf("mem allocation failed.line %d \n", __LINE__);
        return ESP_H264_ERR_MEM;
    }

    if (type == HW)
    {
        ret = esp_h264_enc_hw_new(&cfg, &enc); // has internal memory allocation
    }
    else
    {
        ret = esp_h264_enc_sw_new(&cfg, &enc); // has internal memory allocation
    }
    if (ret != ESP_H264_ERR_OK)
    {
        printf("new failed. line %d \n", __LINE__);
        return ret;
    }

    ret = esp_h264_enc_open(enc); // has internal memory allocation
    if (ret != ESP_H264_ERR_OK)
    {
        printf("open failed .line %d \n", __LINE__);
        return ret;
    }

    return ret;
}

esp_h264_err_t H264_ENC::run_H264_enc_single()
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;

    // need to figure out what read_enc_cb and write_enc_cb do
    int ret_w = read_enc_cb(&in_frame, cfg.res.width, cfg.res.height, cfg.pic_type);
    if (ret_w <= 0)
    {
        printf("read_enc_cb failed. line %d \n", __LINE__);
        return ESP_H264_ERR_FAIL;
    }

    ret = esp_h264_enc_process(enc, &in_frame, &out_frame);
    if (ret != ESP_H264_ERR_OK)
    {
        printf("process failed. line %d \n", __LINE__);
        return ret;
    }
    write_enc_cb(&out_frame);

    return ret;
}

esp_h264_err_t H264_ENC::close_H264_enc_single()
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    ret = esp_h264_enc_close(enc);
    ret = esp_h264_enc_del(enc);
    if (in_frame.raw_data.buffer)
    {
        esp_h264_free(in_frame.raw_data.buffer);
    }
    if (out_frame.raw_data.buffer)
    {
        esp_h264_free(out_frame.raw_data.buffer);
    }
    return ret;
}

esp_h264_enc_cfg_t H264_DEC::set_config_H264_dec_single(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop)
{
    esp_h264_enc_cfg_t config;

    config.pic_type = format;
    config.gop = gop;
    config.fps = fps;
    config.rc = (esp_h264_enc_rc_t){bitrate, qp_min, qp_max};
    config.res = (esp_h264_resolution_t){width, height};

    return config;
}

esp_h264_dec_in_frame_t *H264_DEC::get_inframe()
{
    return &in_frame;
}

esp_h264_dec_out_frame_t *H264_DEC::get_outframe()
{
    return &out_frame;
}

// adapted from esp_h264_sw_dec_test, change calloc function calls
esp_h264_err_t H264_DEC::init_H264_dec_single(esp_h264_enc_cfg_t config)
{
    cfg = config;
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    uint16_t width = ((cfg.res.width + 15) >> 4 << 4); // possibly remove (only in some of example code)
    uint16_t height = ((cfg.res.height + 15) >> 4 << 4);

    in_frame.raw_data.len = (int)((float)width * height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.pic_type));
    in_frame.raw_data.buffer = (uint8_t *)esp_h264_aligned_calloc(16, 1, in_frame.raw_data.len, &in_frame.raw_data.len, ESP_H264_MEM_INTERNAL);
    if (!in_frame.raw_data.buffer)
    {
        printf("mem allocation failed.line %d %d %d %d\n", width, height, (int)in_frame.raw_data.len, __LINE__);
        return ESP_H264_ERR_MEM;
    }

    out_frame.out_size = in_frame.raw_data.len;
    out_frame.outbuf = (uint8_t *)esp_h264_aligned_calloc(16, 1, out_frame.out_size, &out_frame.out_size, ESP_H264_MEM_INTERNAL);
    if (!out_frame.outbuf)
    {
        printf("mem allocation failed.line %d \n", __LINE__);
        return ESP_H264_ERR_MEM;
    }

    esp_h264_dec_cfg_sw_t cfg_;
    cfg_.pic_type = cfg.pic_type;
    ret = esp_h264_dec_sw_new(&cfg_, &dec); // has internal memory allocation
    if (ret != ESP_H264_ERR_OK)
    {
        printf("new failed. line %d \n", __LINE__);
        return ret;
    }

    ret = esp_h264_dec_open(dec);
    if (ret != ESP_H264_ERR_OK)
    {
        printf("open failed .line %d \n", __LINE__);
        return ret;
    }

    return ret;
}

esp_h264_err_t H264_DEC::run_H264_dec_single()
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;

    // need to figure out what read_enc_cb and write_enc_cb do
    int ret_w = read_dec_cd(in_frame.raw_data.buffer, in_frame.raw_data.len, &in_frame);
    if (ret_w <= 0)
    {
        printf("read_dec_cd failed. line %d \n", __LINE__);
        return ESP_H264_ERR_FAIL;
    }

    ret = esp_h264_dec_process(dec, &in_frame, &out_frame);
    if (ret != ESP_H264_ERR_OK)
    {
        printf("process failed. line %d \n", __LINE__);
        return ret;
    }
    write_dec_cd(&out_frame, NULL);

    return ret;
}

esp_h264_err_t H264_DEC::close_H264_dec_single()
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    ret = esp_h264_dec_close(dec);
    ret = esp_h264_dec_del(dec);
    if (in_frame.raw_data.buffer)
    {
        esp_h264_free(in_frame.raw_data.buffer);
    }
    if (out_frame.outbuf)
    {
        esp_h264_free(out_frame.outbuf);
    }
    return ret;
}