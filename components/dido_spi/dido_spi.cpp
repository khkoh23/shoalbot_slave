#include <stdio.h>
#include "dido_spi.h"

DI_DO_SPI::DI_DO_SPI(DI_DO_SPI_config* spi_config) {
    buscfg.mosi_io_num = spi_config->mosi;
    buscfg.miso_io_num = spi_config->miso;
    buscfg.sclk_io_num = spi_config->sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    DiDo_cfg.command_bits = 0;
    DiDo_cfg.address_bits = 0;
    DiDo_cfg.dummy_bits = 0;
    DiDo_cfg.clock_speed_hz = 1000000; // 1 MHz
    DiDo_cfg.duty_cycle_pos = 128;
    DiDo_cfg.mode = 2;
    DiDo_cfg.spics_io_num = spi_config->cs;
    DiDo_cfg.cs_ena_posttrans = 0; 
    DiDo_cfg.cs_ena_pretrans = 0;
    DiDo_cfg.queue_size = 5;

    bus_init = spi_config -> bus_init;
}

esp_err_t DI_DO_SPI::begin() {
    if(!bus_init) {
        ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if(ret != ESP_OK) {
            printf("bus initialization fail, error code: %d\n", ret);
            return ret;
        }
    }
    ret = spi_bus_add_device(SPI3_HOST, &DiDo_cfg, &handle);
    if(ret != ESP_OK) {
        printf("add device failed");
        return ret;
    }
    printf("DI DO initialized");
    return ret;
}

uint8_t DI_DO_SPI::read_di() {
    ret = read_spi(DI_DO_REG::CHAN_STATUS);
    if(ret != ESP_OK) {
        printf("read failed");
        return 0;
    }
    return recvbuf[0];
}

esp_err_t DI_DO_SPI::read_spi(uint8_t reg) {
    t.length = 8;
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;
    sendbuf[0] |= (0b1 << 6);
    ret = spi_device_transmit(handle, &t);
    if(ret != ESP_OK) return ret;
    sendbuf[0] = reg;
    ret = spi_device_transmit(handle, &t);
    sendbuf[0] = 0x00;
    ret = spi_device_transmit(handle, &t);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    if(ret != ESP_OK) return ret;
    return ret;
}