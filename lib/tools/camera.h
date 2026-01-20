#include <pins.h>
#include <Arduino.h>
#include <tvp5151.h>





class camera
{ 


    private:
        int _cam_power_pin;  




    public:
        camera(int cam_power_pin);
        
        void enable_cam_power();



};




