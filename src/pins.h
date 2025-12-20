// have fun!

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
#endif


#ifdef IS_EAGLE
    #define SPI_SCK  9
    #define SPI_MISO 8
    #define SPI_MOSI 10
    #define SI4463_INT 6
    #define SI4463_SDN 5
    #define SI4463_CS 7
#endif