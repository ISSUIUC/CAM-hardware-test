#include <lcd_cam.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_cam_ctlr.h>       // esp code for cam controller
#include "esp_cam_ctlr_types.h" // for defining transaction type
#include "esp_cam_ctlr_dvp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


extern SemaphoreHandle_t Sframe_rdy;
extern esp_cam_ctlr_handle_t cam_handle;

LCD_CAM_Module::LCD_CAM_Module(){}



// Frame signaling/buffers — give external linkage so other C++ files can reference them via `extern`
uint8_t *rx_frame_buf = NULL;
size_t received_frame_size = 0;

static bool IRAM_ATTR dvp_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    rx_frame_buf = (uint8_t *)trans->buffer;
    received_frame_size = trans->received_size;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(Sframe_rdy, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

static bool IRAM_ATTR dvp_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    return false;
}


uint32_t LCD_CAM_Module::read_register_test(uint32_t reg){ // Just a test function 
    return REG_READ(reg);
} 
  


// Read, Modify, Write 
void LCD_CAM_Module::rmw_reg(uint32_t reg, uint32_t clear_mask, uint32_t set_mask){
    uint32_t current = REG_READ(reg);
    current &=  ~clear_mask; 
    current |= set_mask; 
    REG_WRITE(reg, current);

}



esp_err_t LCD_CAM_Module::enable_lcd_cam_controller(){

    // initialize LCD_CAM_Controller

    // Initialize Pin Configuration 

    esp_cam_ctlr_dvp_pin_config pin_cfg{
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            GPIO_NUM_23,
            GPIO_NUM_22,
            GPIO_NUM_21,
            GPIO_NUM_20,
            GPIO_NUM_13,
            GPIO_NUM_12,
            GPIO_NUM_11,
            GPIO_NUM_10,
        },
        .vsync_io = GPIO_NUM_46,
        .de_io = GPIO_NUM_45,
        .pclk_io = GPIO_NUM_2,
        .xclk_io = GPIO_NUM_NC,
    };

    // Initialize Controller Config 

    esp_cam_ctlr_dvp_config_t dvp_config;
    dvp_config.ctlr_id = 0;
    dvp_config.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_config.h_res = 720;
    dvp_config.v_res = 480;
    dvp_config.input_data_color_type = CAM_CTLR_COLOR_YUV422;
    dvp_config.cam_data_width = 8;

    dvp_config.bit_swap_en = 0;
    dvp_config.byte_swap_en = 0;
    dvp_config.bk_buffer_dis = 0;   /*!< Disable backup buffer */
    dvp_config.pin_dont_init = 0;   /*!< Let driver initialize DVP pins and enable clocks */
    dvp_config.pic_format_jpeg = 0; /*!< Input picture format is JPEG, if set this flag and "input_data_color_type" will be ignored */
    dvp_config.external_xtal = 1;

    dvp_config.dma_burst_size = 128;
    dvp_config.xclk_freq = 1;
    dvp_config.pin = &pin_cfg;

    esp_err_t err;
    err = esp_cam_new_dvp_ctlr(&dvp_config, &cam_handle);

    if (err != ESP_OK)
    {
        return err; 
    }


    // set call backs. 
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = dvp_get_new_trans,
        .on_trans_finished = dvp_trans_finished,
    };

    err = esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, NULL);

    if (err != ESP_OK)
    {
        return err; 
    }

    // save internal buffer so on_trans_finished runs

    const void *internal_buf = NULL;
    err = esp_cam_ctlr_get_frame_buffer(cam_handle, 1, &internal_buf);
    if (err != ESP_OK)
    {
        return err; 
    }

    const cam_ctlr_format_conv_config_t conv_cfg = {
        .src_format = CAM_CTLR_COLOR_YUV422, // Source format: YUV422
        .dst_format = CAM_CTLR_COLOR_YUV420, // Destination format: YUV420
        .conv_std = COLOR_CONV_STD_RGB_YUV_BT601,
        .data_width = 8,
        .input_range = COLOR_RANGE_LIMIT,
        .output_range = COLOR_RANGE_LIMIT,
    };
    
    ESP_ERROR_CHECK(esp_cam_ctlr_format_conversion(cam_handle, &conv_cfg));

    // Serial.println("Enable CAM Ctlr");

    esp_err_t err2;
    err2 = esp_cam_ctlr_enable(cam_handle); // enable high peripheral // no work

    if (err2 != ESP_OK)
    {
        return err2;
    }

    esp_err_t err3;
    err3 = esp_cam_ctlr_start(cam_handle);

    if (err3 != ESP_OK)
    {
        return err3;
    }


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
void LCD_CAM_Module::set_width_input_data(bool set){
    if(set){
        // Set width input data 8 bits.
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_2BYTE_EN, LCDCAM_CAM_2BYTE_EN);
    }

}


//LCD_CAM_CAM_START Camera module start signal. || (R/W) 
// 0: -> Default (OFF) (FALSE)
// 1: -> ON (TRUE)
void LCD_CAM_Module::initialize_cam_ctrl(bool set){
    if(set){
        // Initialize cam controller 
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_START, LCDCAM_CAM_START);
    }
    else{
        // clear CAM Start bit. 
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_START, 0);

    }
}


// LCD_CAM_CAM_RESET When set to 1, the Camera module is reset. (WO)

void LCD_CAM_Module::reset_cam_ctrl(bool reset){
    if(reset){
        // reset cam controller 
        rmw_reg(LCDCAM_CAM_CTRL1_REG, LCDCAM_CAM_RESET, LCDCAM_CAM_RESET);
    }
}


void LCD_CAM_Module::full_reboot_cam_ctrl(){
    initialize_cam_ctrl(false); // stop cam controller 

    reset_cam_ctrl(true); // reset cam controller

    initialize_cam_ctrl(true); // turn on cam controller
}

// LCD_CAM_CAM_VH_DE_MODE_EN Configures the input control signals. (Bit 28)
// 0: VSYNC and DE signals control the data. In this case, wiring HSYNC signal line is not a must.
// But in this case, the YUV-RGB conversion function of the camera module is not available.
// 1: VSYNC, HSYNC, and DE signals control the data. In this case, users need to wire the three
// signal lines.
// (R/W)


void LCD_CAM_Module::toggle_lcd_cam_vh_de_mode(bool toggle){
    // ESP_ERROR_CHECK()



}


// /**
//  * @brief Combine a GPIO input with a peripheral signal, which tagged as input attribute.
//  *
//  * @note There's no limitation on the number of signals that a GPIO can combine with.
//  *
//  * @param gpio_num GPIO number, especially, `GPIO_MATRIX_CONST_ZERO_INPUT` means connect logic 0 to signal
//  *                                          `GPIO_MATRIX_CONST_ONE_INPUT` means connect logic 1 to signal
//  * @param signal_idx Peripheral signal index (tagged as input attribute)
//  * @param inv  Whether the GPIO input to be inverted or not
//  */
// void esp_rom_gpio_connect_in_signal(uint32_t gpio_num, uint32_t signal_idx, bool inv);



static esp_err_t cam_config_gpio(uint32_t gpio, uint32_t index){

    // configure gpio pins
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t err = gpio_config(&cfg); // the address / location in memory of cfg
    if (err!=ESP_OK){
        return err;
    }

    // set gpio p in to assigned index
    esp_rom_gpio_connect_in_signal(gpio, index, false);

    return ESP_OK;
}


esp_err_t LCD_CAM_Module::cam_controller_configure_gpio_matrix(){

    esp_err_t err; 

    // Toggle Camera Input
    for (int i = 0; i < 8; i++)
    {
    err = cam_config_gpio(YOUT[i], YOUT_SIGNAL_INDEX[i]);
    if(err!=ESP_OK){
        return err;
    }
    }

    // Toggle H_Sync Input
    err= cam_config_gpio(TVP5151_HSYNC, CAM_H_SYNC_PAD_IN_IDX);
    if(err!=ESP_OK){
        return err;
    }

    // Toggle V_Sync Input
    err =cam_config_gpio(TVP5151_VSYNC, CAM_V_SYNC_PAD_IN_IDX);
    if(err!=ESP_OK){
        return err;
    }

    // Toggle Pixel Clock Input
    err =cam_config_gpio(TVP5151_SCLK, CAM_PCLK_PAD_IN_IDX);
    if(err!=ESP_OK){
        return err;
    }

    // Toggle DE_Signal Input  (Connected to active video signal)
     err =cam_config_gpio(TVP5151_AVID, CAM_H_ENABLE_PAD_IN_IDX);
    if(err!=ESP_OK){
        return err;
    }    

    return ESP_OK;

}

