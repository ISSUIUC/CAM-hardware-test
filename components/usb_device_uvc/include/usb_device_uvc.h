/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <sys/time.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief UVC format
     */
    typedef enum
    {
        UVC_FORMAT_JPEG, /*!< JPEG format */
        UVC_FORMAT_H264, /*!< H264 format */
    } uvc_format_t;

    /**
     * @brief Frame buffer structure
     */
    typedef struct
    {
        uint8_t *buf;             /*!< Pointer to the frame data */
        size_t len;               /*!< Length of the buffer in bytes */
        size_t width;             /*!< Width of the image frame in pixels */
        size_t height;            /*!< Height of the image frame in pixels */
        uvc_format_t format;      /*!< Format of the frame data */
        struct timeval timestamp; /*!< Timestamp since boot of the frame */
    } uvc_fb_t;

    /**
     * @brief type of callback function when host open the UVC device
     */
    typedef esp_err_t (*uvc_input_start_cb_t)(uvc_format_t format, int width, int height, int rate, void *cb_ctx);

    /**
     * @brief type of callback function when host request a new frame buffer
     */
    typedef uvc_fb_t *(*uvc_input_fb_get_cb_t)(void *cb_ctx);

    /**
     * @brief type of callback function when the frame buffer is no longer used
     */
    typedef void (*uvc_input_fb_return_cb_t)(uvc_fb_t *fb, void *cb_ctx);

    /**
     * @brief type of callback function when host close the UVC device
     */
    typedef void (*uvc_input_stop_cb_t)(void *cb_ctx);

    /**
     * @brief Configuration for the UVC device
     */
    typedef struct
    {
        uint8_t *uvc_buffer;                   /*!< UVC transfer buffer */
        uint32_t uvc_buffer_size;              /*!< UVC transfer buffer size, should bigger than one frame size */
        uvc_input_start_cb_t start_cb;         /*!< callback function of host open the UVC device with the specific format and resolution */
        uvc_input_fb_get_cb_t fb_get_cb;       /*!< callback function of host request a new frame buffer */
        uvc_input_fb_return_cb_t fb_return_cb; /*!< callback function of the frame buffer is no longer used */
        uvc_input_stop_cb_t stop_cb;           /*!< callback function of host close the UVC device */
        void *cb_ctx;                          /*!< callback context, for user specific usage */
    } uvc_device_config_t;

    /**
     * @brief Configure the UVC device by uvc device number
     *
     * @param index UVC device index number [0,1]
     * @param config  Configuration for the UVC device
     * @return ESP_OK on success
     *         ESP_ERR_INVALID_ARG if the configuration is invalid
     *         ESP_FAIL if the UVC device could not be initialized
     */
    esp_err_t uvc_device_config(int index, uvc_device_config_t *config);

    /**
     * @brief Initialize the UVC device, after this function is called, the UVC device will be visible to the host
     *       and the host can open the UVC device with the specific format and resolution.
     *
     * @return ESP_OK on success
     *         ESP_FAIL if the UVC device could not be initialized
     */
    esp_err_t uvc_device_init(void);

    /**
     * @brief Deinitialize the UVC device
     *
     * @return ESP_OK on success
     *         ESP_ERR_INVALID_STATE if the UVC device is not initialized, or event group is NULL
     *
     */
    esp_err_t uvc_device_deinit(void);

#ifdef __cplusplus
}
#endif

/*
Example config file:
https://github.com/espressif/esp-iot-solution/blob/a558bfb49d2b61e4361e748d446b20f1b5f93c65/examples/usb/device/usb_dual_uvc_device/sdkconfig.ci.esp32p4_h264#L7
# This file was generated using idf.py save-defconfig. It can be edited manually.
# Espressif IoT Development Framework (ESP-IDF) 5.2.1 Project Minimal Configuration
#

CONFIG_IDF_TARGET="esp32p4"
CONFIG_UVC_SUPPORT_TWO_CAM=y
CONFIG_UVC_MODE_BULK_CAM1=y
CONFIG_FORMAT_H264_CAM1=y
CONFIG_UVC_CAM1_FRAMERATE=15
CONFIG_UVC_CAM1_FRAMESIZE_WIDTH=1280
CONFIG_UVC_CAM1_FRAMESIZE_HEIGT=720
CONFIG_UVC_CAM1_MULTI_FRAMESIZE=n
CONFIG_UVC_MODE_BULK_CAM2=y
CONFIG_UVC_CAM2_MULTI_FRAMESIZE=n

#
# ESP PSRAM
#
CONFIG_SPIRAM=y
CONFIG_SPIRAM_SPEED_200M=y
CONFIG_IDF_EXPERIMENTAL_FEATURES=y
*/