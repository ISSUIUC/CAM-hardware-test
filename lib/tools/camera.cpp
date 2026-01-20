
#include <camera.h>



camera::camera(int cam_power_pin){
    _cam_power_pin = cam_power_pin;
}



// provides power to camera
void camera::enable_cam_power(){

    pinMode(LED_RED, OUTPUT);
    pinMode(_cam_power_pin, OUTPUT);
    digitalWrite(_cam_power_pin, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(_cam_power_pin, HIGH);
    digitalWrite(LED_RED, HIGH);
    delay(10000);

}



