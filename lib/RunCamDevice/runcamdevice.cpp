#include <runcamdevice.h>

bool RCDevice2Button::on_off() {
    constexpr auto kOnOffCmd = rc_command<0xCC, 0x01, 0x01, 0xE7>();
}