#include <Arduino.h>

#include "usb_device_uvc.h"

class UVC_device
{
private:
    uint8_t *uvc_buf;

public:
    void init();
    void deinit();
};