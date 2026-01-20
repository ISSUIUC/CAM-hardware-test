#include <Arduino.h>
#include "venc.h"
#include "camera.h"
#include "pins.h"
#include "lcd_cam.h"
#include "usb_device_uvc.h"
#include <esp_cam_ctlr.h>       // esp code for cam controller
#include "esp_cam_ctlr_types.h" // for defining transaction type
#include "esp_cam_ctlr_dvp.h"

class UVC_device
{
private:
    uint8_t *uvc_buf;

public:
    void init();
    void deinit();
};