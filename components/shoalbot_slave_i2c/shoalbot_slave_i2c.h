#include "driver/i2c_slave.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <cstring>
#include "esp_log.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint16_t slaveAddr;
} i2c_slave_config;


//static const char *TAG = "i2c-slave";

static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

class shoalbot_slave_i2c {
    public:
        shoalbot_slave_i2c(i2c_slave_config* slave_config);
        esp_err_t i2c_read(); // read DO cmd from master
        esp_err_t i2c_send_state(uint8_t* data, uint8_t index);
        bool get_state();
        bool get_do();
        uint16_t return_NavState();
        uint16_t return_DO();
        uint8_t return_BMS();
        void set_state(uint8_t* state_data);
//        bool get_batSW();
//        void set_batSW(bool batsw);
 
    private:
        i2c_slave_config_t slv_conf = {};
        i2c_slave_dev_handle_t slv_handle;
        i2c_slave_event_callbacks_t cbs;
        i2c_slave_rx_done_event_data_t rx_data;
        esp_err_t ret;  
        SemaphoreHandle_t i2c_mutex = xSemaphoreCreateBinary();

        uint8_t DO_cnt = 0;
        uint16_t parsed_data_DO = 0;
        uint16_t parsed_data_NavState = 0;

        uint8_t parsed_data_BMS = 0;

        uint8_t current_State[5] = {};

        uint8_t batSW = 0;

        bool DO_ack_flag = false;
        bool State_ack_flag = false;
        bool batSW_ack_flag = false;

        bool new_State = false;
        bool new_DO = false;
        bool new_batSW = false;
};