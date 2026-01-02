#include <lcd_cam_reg.h>


#include <stdint.h>
#include <stdbool.h>


uint32_t read_register(uint32_t reg){ // Just a test function 
    return REG_READ(reg);
} 


// Read, Modify, Write 
void rmw_reg(uint32_t reg, uint32_t clear_mask, uint32_t set_mask){
    uint32_t current = REG_READ(reg);
    current &=  ~clear_mask; 
    current |= set_mask; 
    REG_WRITE(reg, current);
}


// Register 9.43. IO_MUX_GPIOn_REG (n: 0 - 54) (0x0004+4*n)




// IO_MUX_GPIOn_MCU_SEL Configures to select IO MUX function for this pin. || Bit 12-14
// 0: Select Function 0
// 1: Select Function 1
// ......
// (R/W)






// Register 36.13. LCD_CAM_CAM_CTRL1_REG (0x0008) (Begins line 170 in lcd_cam_reg.h)

// LCD_CAM_CAM_2BYTE_EN Configures the width of input data. || Bit 24 || (R/W)
// 0: 8 bits. (FALSE) -> Default
// 1: 16 bits. (TRUE)
void set_width_input_data(bool set){
    if(set){
        // Set width input data 8 bits.
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_VSYNC_FILTER_EN, LCDCAM_CAM_VSYNC_FILTER_EN);
    }

}


//LCD_CAM_CAM_START Camera module start signal. || (R/W) 
// 0: -> Default (OFF) (FALSE)
// 1: -> ON (TRUE)
void initialize_cam_ctrl(bool set){
    if(set){
        // Initialize cam controller 
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_START, LCDCAM_CAM_START);
    }
}


// LCD_CAM_CAM_RESET When set to 1, the Camera module is reset. (WO)

void reset_cam_ctrl(bool reset){
    if(reset){
        // reset cam controller 
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_RESET, LCDCAM_CAM_RESET);
    }
}



