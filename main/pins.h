
#pragma once 


#ifdef IS_CAM
    #define SPI_SCK  9
    #define SPI_MISO 7
    #define SPI_MOSI 8
    #define SI4463_INT 5
    #define SI4463_SDN 4
    #define SI4463_CS 6

    #define I2C_SDA 49
    #define I2C_SCL 50
    #define TVP5151_ADDR 0x5C

    #define TVP5151_PDN 25
    #define TVP5151_RESET 3
    #define TVP5151_VSYNC 46
    #define TVP5151_HSYNC 41
    #define TVP5151_SCLK 2
    #define TVP5151_AVID 45

    #define LED_BLUE 54
    #define LED_GREEN 53
    #define LED_ORANGE 52
    #define LED_RED 51

    #define CAM1_ON_OFF 31
    #define CAM2_ON_OFF 29
    #define CAM1_RX 30
    #define CAM1_TX 28
    #define CAM2_RX 33
    #define CAM2_TX 32

    

    #define BUZZER_PIN 24

    constexpr uint8_t YOUT[8] = {23, 22, 21, 20, 13, 12, 11, 10};
#endif


#ifdef IS_EAGLE
    #define SPI_SCK  9
    #define SPI_MISO 8
    #define SPI_MOSI 10
    #define SI4463_INT 6
    #define SI4463_SDN 5
    #define SI4463_CS 7
#endif