#include "venc.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_h264_alloc.h"
#include "../../components/esp_h264/test_apps/main/h264_io.h"

esp_h264_enc_cfg_hw_t H264::set_hw_H264_config_single(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop) {
    esp_h264_enc_cfg_hw_t config;

    config.pic_type = format;
    config.gop = gop;
    config.fps = fps;
    config.rc = (esp_h264_enc_rc_t) {bitrate, qp_min, qp_max};
    config.res = (esp_h264_resolution_t) {width, height};

    return config;
}

esp_h264_enc_cfg_dual_hw_t H264::set_hw_H264_config_dual(esp_h264_raw_format_t format, uint8_t fps, uint16_t height, uint16_t width, uint32_t bitrate, uint8_t qp_min, uint8_t qp_max, uint8_t gop) {
    esp_h264_enc_cfg_dual_hw_t config;

    config.cfg0.pic_type = format;
    config.cfg0.gop = gop;
    config.cfg0.fps = fps;
    config.cfg0.rc = (esp_h264_enc_rc_t) {bitrate, qp_min, qp_max};
    config.cfg0.res = (esp_h264_resolution_t) {width, height};

    config.cfg1.pic_type = format;
    config.cfg1.gop = gop;
    config.cfg1.fps = fps;
    config.cfg1.rc = (esp_h264_enc_rc_t) {bitrate, qp_min, qp_max};
    config.cfg1.res = (esp_h264_resolution_t) {width, height};

    return config;
}

//adapted from esp_h264_hw_enc_test.c (thread functions), need to add source for frames + remove calloc/free for static buf + remove goto
esp_h264_err_t H264::run_hw_H264_enc_single(esp_h264_enc_cfg_hw_t cfg) {
    esp_h264_enc_in_frame_t in_frame = {0};
    esp_h264_enc_out_frame_t out_frame = {0};
    esp_h264_err_t ret = ESP_H264_ERR_FAIL;
    esp_h264_enc_handle_t enc = NULL;
    esp_h264_resolution_t res;
    esp_h264_enc_rc_t rc;
    uint32_t frame_count = 0;
    uint8_t gop;
    uint8_t fps;
    esp_h264_enc_param_hw_handle_t param_hd;
    int index_c = 0;

    in_frame.raw_data.len = (int)((float)cfg.res.width * cfg.res.height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.pic_type));
    in_frame.raw_data.buffer = esp_h264_aligned_calloc(16, 1, in_frame.raw_data.len, &in_frame.raw_data.len, ESP_H264_MEM_SPIRAM);
    if (!in_frame.raw_data.buffer) {
        printf("mem allocation failed.line %d \n", __LINE__);
        goto _exit_;
    }
    out_frame.raw_data.len = (int)((float)cfg.res.width * cfg.res.height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.pic_type)) / 10;
    out_frame.raw_data.buffer = esp_h264_aligned_calloc(16, 1, out_frame.raw_data.len, &out_frame.raw_data.len, ESP_H264_MEM_SPIRAM);
    if (!out_frame.raw_data.buffer) {
        printf("mem allocation failed.line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_hw_new(&cfg, &enc);
    if (ret != ESP_H264_ERR_OK) {
        printf("new failed. line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_hw_get_param_hd(enc, &param_hd);
    if (ret != ESP_H264_ERR_OK) {
        printf("esp_h264_enc_hw_get_param_hd error. line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_get_resolution(&param_hd->base, &res);
    if ((ret != ESP_H264_ERR_OK)
            || (res.width != cfg.res.width)
            || (res.height != cfg.res.height)) {
        printf("esp_h264_enc_get_resolution failed .line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_get_fps(&param_hd->base, &fps);
    if ((ret != ESP_H264_ERR_OK)
            || (fps != cfg.fps)) {
        printf("esp_h264_enc_get_fps failed .line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_get_gop(&param_hd->base, &gop);
    if ((ret != ESP_H264_ERR_OK)
            || (gop != cfg.gop)) {
        printf("esp_h264_enc_get_gop failed .line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_get_bitrate(&param_hd->base, &rc.bitrate);
    if (ret != ESP_H264_ERR_OK
            || (rc.bitrate != cfg.rc.bitrate)) {
        printf("esp_h264_enc_get_bitrate failed .line %d \n", __LINE__);
        goto _exit_;
    }

    ret = esp_h264_enc_open(enc);
    if (ret != ESP_H264_ERR_OK) {
        printf("open failed .line %d \n", __LINE__);
        goto _exit_;
    }
    while (1) {
        index_c++;
        int ret_w = read_enc_cb(&in_frame, cfg.res.width, cfg.res.height, cfg.pic_type);
        if (ret_w <= 0) {
            break;
        }
        ret |= esp_h264_enc_get_resolution(&param_hd->base, &res);
        if ((ret != ESP_H264_ERR_OK)
                || (res.width != cfg.res.width)
                || (res.height != cfg.res.height)) {
            printf("esp_h264_enc_get_resolution failed .line %d \n", __LINE__);
            goto _exit_;
        }

        ret |= esp_h264_enc_get_fps(&param_hd->base, &fps);
        if ((ret != ESP_H264_ERR_OK)
                || (fps != cfg.fps)) {
            printf("esp_h264_enc_get_fps failed .line %d \n", __LINE__);
            goto _exit_;
        }

        ret |= esp_h264_enc_get_gop(&param_hd->base, &gop);
        if ((ret != ESP_H264_ERR_OK)
                || (gop != cfg.gop)) {
            printf("esp_h264_enc_get_gop failed .line %d \n", __LINE__);
            goto _exit_;
        }

        ret |= esp_h264_enc_get_bitrate(&param_hd->base, &rc.bitrate);
        if (ret != ESP_H264_ERR_OK
                || (rc.bitrate != cfg.rc.bitrate)) {
            printf("esp_h264_enc_get_bitrate failed .line %d \n", __LINE__);
            goto _exit_;
        }

        cfg.fps = index_c + 4;
        ret |= esp_h264_enc_set_fps(&param_hd->base, cfg.fps);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_fps failed .line %d \n", __LINE__);
            goto _exit_;
        }
        cfg.gop = index_c + 3;
        ret |= esp_h264_enc_set_gop(&param_hd->base, cfg.gop);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_gop failed .line %d \n", __LINE__);
            goto _exit_;
        }
        frame_count = 0;
        cfg.rc.qp_min = index_c + 3;
        cfg.rc.qp_max = index_c + 10;
        ret |= esp_h264_enc_set_bitrate(&param_hd->base, cfg.rc.bitrate);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_bitrate failed .line %d \n", __LINE__);
            goto _exit_;
        }
        ret |= esp_h264_enc_process(enc, &in_frame, &out_frame);
        if (ret != ESP_H264_ERR_OK) {
            printf("process failed. line %d \n", __LINE__);
            goto _exit_;
        }
        write_enc_cb(&out_frame);
        if (frame_count % cfg.gop == 0) {
            if (out_frame.frame_type != ESP_H264_FRAME_TYPE_I
                    && out_frame.frame_type != ESP_H264_FRAME_TYPE_IDR) {
                printf("frame type error. frame type %d GOP %d line %d \n", out_frame.frame_type, cfg.gop, __LINE__);
                ret = ESP_H264_ERR_FAIL;
                goto _exit_;
            }
        } else {
            if (out_frame.frame_type != ESP_H264_FRAME_TYPE_P) {
                printf("frame type error. frame type %d GOP %d frame count %ld line %d \n", out_frame.frame_type, cfg.gop, frame_count, __LINE__);
                ret = ESP_H264_ERR_FAIL;
                goto _exit_;
            }
        }
        frame_count++;
    }
_exit_:
    ret |= esp_h264_enc_close(enc);
    ret |= esp_h264_enc_del(enc);
    if (in_frame.raw_data.buffer) {
        esp_h264_free(in_frame.raw_data.buffer);
    }
    if (out_frame.raw_data.buffer) {
        esp_h264_free(out_frame.raw_data.buffer);
    }
    return ret;


}

esp_h264_err_t H264::run_hw_H264_enc_double(esp_h264_enc_cfg_dual_hw_t cfg) {
    esp_h264_err_t ret = ESP_H264_ERR_FAIL;
    int index_c = 0;
    esp_h264_resolution_t res;
    esp_h264_enc_rc_t rc;
    uint8_t gop[2] = { cfg.cfg0.gop, cfg.cfg1.gop };
    uint8_t gop_tmp;
    uint8_t fps;
    uint32_t frame_count = 0;
    esp_h264_enc_param_hw_handle_t param_hd;
    esp_h264_enc_param_hw_handle_t param_hd0;
    esp_h264_enc_param_hw_handle_t param_hd1;
    esp_h264_enc_in_frame_t *in_frame[2] = {NULL, NULL};
    esp_h264_enc_out_frame_t *out_frame[2] = {NULL, NULL};
    esp_h264_enc_dual_handle_t enc = NULL;
    int16_t out_length[2];
    esp_h264_enc_cfg_t base_cfg;
    int16_t width[2] = { cfg.cfg0.res.width, cfg.cfg1.res.width };
    int16_t height[2] = { cfg.cfg0.res.height, cfg.cfg1.res.height };
    out_length[0] = (int)((float)cfg.cfg0.res.width * cfg.cfg0.res.height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.cfg0.pic_type));
    out_length[1] = (int)((float)cfg.cfg1.res.width * cfg.cfg1.res.height * ESP_H264_GET_BPP_BY_PIC_TYPE(cfg.cfg1.pic_type));

    for (int16_t i = 0; i < 2; i++) {
        in_frame[i] = heap_caps_calloc(1, sizeof(esp_h264_enc_in_frame_t), MALLOC_CAP_INTERNAL);
        if (!in_frame[i]) {
            printf("mem allocation failed.line %d \n", __LINE__);
            goto _exit_dual_;
        }
        out_frame[i] = heap_caps_calloc(1, sizeof(esp_h264_enc_out_frame_t), MALLOC_CAP_INTERNAL);
        if (!out_frame[i]) {
            printf("mem allocation failed.line %d \n", __LINE__);
            goto _exit_dual_;
        }
        in_frame[i]->raw_data.len = out_length[i];
        in_frame[i]->raw_data.buffer = esp_h264_aligned_calloc(16, 1, in_frame[i]->raw_data.len, &in_frame[i]->raw_data.len, ESP_H264_MEM_INTERNAL);
        if (!in_frame[i]->raw_data.buffer) {
            printf("mem allocation failed.line %d \n", __LINE__);
            goto _exit_dual_;
        }
        out_frame[i]->raw_data.len = out_length[i];
        out_frame[i]->raw_data.buffer = esp_h264_aligned_calloc(16, 1, out_frame[i]->raw_data.len, &out_frame[i]->raw_data.len, ESP_H264_MEM_INTERNAL);
        if (!out_frame[i]->raw_data.buffer) {
            printf("mem allocation failed. line %d \n", __LINE__);
            goto _exit_dual_;
        }
        out_length[i] = out_frame[i]->raw_data.len;
    }

    ret = esp_h264_enc_dual_hw_new(&cfg, &enc);
    if (ret != ESP_H264_ERR_OK) {
        printf("new failed. line %d \n", __LINE__);
        goto _exit_dual_;
    }

    ret = esp_h264_enc_dual_hw_get_param_hd0(enc, &param_hd0);
    if (ret != ESP_H264_ERR_OK) {
        printf("esp_h264_enc_hw_get_param_hd error. line %d \n", __LINE__);
        goto _exit_dual_;
    }

    ret = esp_h264_enc_dual_hw_get_param_hd1(enc, &param_hd1);
    if (ret != ESP_H264_ERR_OK) {
        printf("esp_h264_enc_hw_get_param_hd error. line %d \n", __LINE__);
        goto _exit_dual_;
    }

    param_hd = param_hd0;
    ret = esp_h264_enc_dual_open(enc);
    if (ret != ESP_H264_ERR_OK) {
        printf("open failed .line %d \n", __LINE__);
        goto _exit_dual_;
    }
    while (1) {
        index_c++;
        param_hd = index_c % 2 ? param_hd0 : param_hd1;
        memcpy(&base_cfg, index_c % 2 ? &cfg.cfg0 : &cfg.cfg1, sizeof(esp_h264_enc_cfg_t));
        for (int16_t i = 0; i < 2; i++) {
            esp_h264_enc_cfg_hw_t cfg_tmp = i == 0 ? cfg.cfg0 : cfg.cfg1;
            int ret_w = read_enc_cb(in_frame[i], width[i], height[i], cfg_tmp.pic_type);
            if (ret_w <= 0) {
                goto _exit_dual_;
            }
        }

        base_cfg.fps = index_c + 4;
        ret = esp_h264_enc_set_fps(&param_hd->base, base_cfg.fps);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_fps failed .line %d \n", __LINE__);
            goto _exit_dual_;
        }
        gop[index_c % 2] = (index_c + 3);
        ret = esp_h264_enc_set_gop(&param_hd->base, gop[index_c % 2]);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_gop failed .line %d \n", __LINE__);
            goto _exit_dual_;
        }
        frame_count = 0;

        base_cfg.rc.qp_min = index_c + 3;
        base_cfg.rc.qp_max = index_c + 10;
        base_cfg.rc.bitrate = base_cfg.res.width * base_cfg.res.height / 20;
        ret = esp_h264_enc_set_bitrate(&param_hd->base, base_cfg.rc.bitrate);
        if (ret != ESP_H264_ERR_OK) {
            printf("esp_h264_enc_set_bitrate failed .line %d \n", __LINE__);
            goto _exit_dual_;
        }
        ret = esp_h264_enc_dual_process(enc, in_frame, out_frame);
        if (ret != ESP_H264_ERR_OK) {
            printf("process failed. line %d \n", __LINE__);
            goto _exit_dual_;
        }
        for (int16_t i = 0; i < 2; i++) {
            write_enc_cb(out_frame[i]);
            for (int16_t i = 0; i < 2; i++) {
                if (frame_count % base_cfg.gop == 0) {
                    if (out_frame[i]->frame_type != ESP_H264_FRAME_TYPE_I) {
                        printf("frame type error. frame type %d GOP %d line %d \n", out_frame[i]->frame_type, base_cfg.gop, __LINE__);
                        goto _exit_dual_;
                    }
                } else {
                    if (out_frame[i]->frame_type != ESP_H264_FRAME_TYPE_P) {
                        printf("frame type error. frame type %d GOP %d line %d \n", out_frame[i]->frame_type, base_cfg.gop, __LINE__);
                        goto _exit_dual_;
                    }
                }
            }
        }
        frame_count++;
        ret = esp_h264_enc_get_resolution(&param_hd->base, &res);
        if ((ret != ESP_H264_ERR_OK)
                || (res.width != base_cfg.res.width)
                || (res.height != base_cfg.res.height)) {
            printf("esp_h264_enc_get_resolution failed .line %d \n", __LINE__);
            goto _exit_dual_;
        }

        ret = esp_h264_enc_get_fps(&param_hd->base, &fps);
        if ((ret != ESP_H264_ERR_OK)
                || (fps != base_cfg.fps)) {
            printf("esp_h264_enc_get_fps failed .line %d \n", __LINE__);
            goto _exit_dual_;
        }

        ret = esp_h264_enc_get_gop(&param_hd->base, &gop_tmp);

        if ((ret != ESP_H264_ERR_OK)
                || (gop_tmp != ((gop[0] + gop[1]) >> 1)
                    && gop_tmp != gop[index_c % 2])) {
            printf("esp_h264_enc_get_gop failed . %d %d %d %d line %d \n", index_c, gop[0], gop[1], gop_tmp, __LINE__);
            goto _exit_dual_;
        }
        ret = esp_h264_enc_get_bitrate(&param_hd->base, &rc.bitrate);
        if (ret != ESP_H264_ERR_OK
                || (rc.bitrate != base_cfg.rc.bitrate)) {
            printf("esp_h264_enc_get_bitrate failed .line %d \n", __LINE__);
            printf("bitrate %d %d \n", (int)rc.bitrate, (int)base_cfg.rc.bitrate);
            goto _exit_dual_;
        }
    }
_exit_dual_:
    ret |= esp_h264_enc_dual_close(enc);
    ret |= esp_h264_enc_dual_del(enc);
    for (int16_t i = 0; i < 2; i++) {
        if (in_frame[i]) {
            if (in_frame[i]->raw_data.buffer) {
                esp_h264_free(in_frame[i]->raw_data.buffer);
                esp_h264_free(in_frame[i]);
            }
        }
        if (out_frame[i]) {
            if (out_frame[i]->raw_data.buffer) {
                esp_h264_free(out_frame[i]->raw_data.buffer);
                esp_h264_free(out_frame[i]);
            }
        }
    }
    return ret;
}