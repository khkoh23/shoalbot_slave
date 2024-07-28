#ifndef DI_DO_SPI_H
#define DI_DO_SPI_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "esp_system.h"

#define CID 0x0

typedef struct {
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sclk;
    gpio_num_t cs;
    bool bus_init;
} DI_DO_SPI_config;

enum DI_DO_REG : uint8_t {
    CHAN_STATUS = 0x00, // Current value of each of the eight input channels {STATUS[7:0]}
    DBNC_MODE0 = 0x10, // Mode control bits for the first four channel debounce filters organized as: {md_ch3[1:0],md_ch2[1:0],md_ch1[1:0],md_ch0[1:0]}
    DBNC_MODE1 = 0x20, // Mode control bits for the second four channel debounce filters organized as: {md_ch7[1:0],md_ch6[1:0],md_ch5[1:0],md_ch4[1:0]}
    DBNC_DLY0 = 0x30, // Delay control bits for the first four channel debounce filters organized as: {dly_ch3[1:0],dly_ch2[1:0],dly_ch1[1:0],dly_ch0[1:0]}
    DBNC_DLY1 = 0x40, // Delay control bits for the second four channel debounce filters organized as: {dly_ch7[1:0],dly_ch6[1:0],dly_ch5[1:0],dly_ch4[1:0]}
};

class DI_DO_SPI {
    public:
        DI_DO_SPI(DI_DO_SPI_config* spi_config);
        esp_err_t begin();
        uint8_t read_di();
        

    private:
        esp_err_t ret;  
        spi_device_handle_t handle; 
        spi_bus_config_t buscfg = {};
        spi_device_interface_config_t DiDo_cfg = {};
        spi_transaction_t t = {};

        uint8_t sendbuf[1] = {0x00};
        uint8_t recvbuf[1] = {0x00};

        esp_err_t read_spi(uint8_t reg);

        bool bus_init = false;
};





#endif // DI_DO_SPI_H