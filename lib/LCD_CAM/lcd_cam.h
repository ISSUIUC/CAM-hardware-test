#include <lcd_cam_reg.h>
#include <lcd_cam_struct.h>
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"
#include "esp_rom_gpio.h" 
#include "../../main/pins.h"


class LCD_CAM_Module { 

    private:

        // indexs for GPIO Matrix and IO Mux Config
        static constexpr uint8_t YOUT_SIGNAL_INDEX[8] = {CAM_DATA_IN_PAD_IN0_IDX, CAM_DATA_IN_PAD_IN1_IDX,
            CAM_DATA_IN_PAD_IN2_IDX, CAM_DATA_IN_PAD_IN3_IDX, CAM_DATA_IN_PAD_IN4_IDX,
            CAM_DATA_IN_PAD_IN5_IDX, CAM_DATA_IN_PAD_IN6_IDX, CAM_DATA_IN_PAD_IN7_IDX};



        // static constexpr uint8_t CAM_HSYNC_SIGNAL = CAM_H_SYNC_PAD_IN_IDX;
        // static constexpr uint8_t CAM_VSYNC_SIGNAL = CAM_V_SYNC_PAD_IN_IDX;
        // static constexpr uint8_t CAM_PCLK_SIGNAL = CAM_PCLK_PAD_IN_IDX; // SCLK as this advances the bus (2 Bits = 1 Pixel) Models SCLK models bits not pixels
        // static constexpr uint8_t CAM_DE_SIGNAL = CAM_H_ENABLE_PAD_IN_IDX; // set high data always enabled no input from tvp


        
    public: 
        
        LCD_CAM_Module();

        // Test
        uint32_t read_register_test(uint32_t reg);

        // Tools
        void rmw_reg(uint32_t reg, uint32_t clear_mask, uint32_t set_mask);



        // Setting LCD_CAM
        void set_width_input_data(bool set);
        void initialize_cam_ctrl(bool set);
        void reset_cam_ctrl(bool reset);
        void full_reboot_cam_ctrl();
        void toggle_lcd_cam_vh_de_mode(bool toggle);

        // Setting GPIO Matrix
        esp_err_t cam_controller_configure_gpio_matrix();
    


};
