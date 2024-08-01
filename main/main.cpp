//#include <iostream>
//#include <stdlib.h>
//#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
//#include <chrono>
#include "dido_spi.h"
#include "shoalbot_slave_i2c.h"
#include "dmx_485.h"

#define JETSON_24 GPIO_NUM_0
#define DI_1 GPIO_NUM_1
#define DI_0 GPIO_NUM_2
#define DO_4 GPIO_NUM_3	
#define DI_13 GPIO_NUM_4
#define DI_14 GPIO_NUM_5
#define DI_15 GPIO_NUM_6
#define DI_16 GPIO_NUM_7
#define RS3_DE GPIO_NUM_8
#define SPI_CS_1 GPIO_NUM_9 // si8380 (DI10,11,17,19,20-23)
#define SPI_CS_2 GPIO_NUM_10 // si8380 (DI2-9)
#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define PASS_1 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS3_TX GPIO_NUM_17
#define RS3_RX GPIO_NUM_18
#define DI_18 GPIO_NUM_19
#define DI_12 GPIO_NUM_20
#define PASS_2 GPIO_NUM_21
#define DO_0 GPIO_NUM_35	// 1
#define DO_1 GPIO_NUM_36	// 2
#define DO_2 GPIO_NUM_37	// 4
#define DO_3 GPIO_NUM_38	// 8
#define DO_6 GPIO_NUM_39	// 64
#define DO_7 GPIO_NUM_40	// 128
#define DO_8 GPIO_NUM_41	// 256
#define DO_9 GPIO_NUM_42	// 512
#define DO_5 GPIO_NUM_46	// 32
#define BOOTKEY GPIO_NUM_47	
#define BMS GPIO_NUM_48

//#define GPIO_INPUT_PIN_SEL  ((1ULL<<DI_0) | (1ULL<<DI_1) | (1ULL<<BAT_SW))
//#define ESP_INTR_FLAG_DEFAULT 0

//static QueueHandle_t estop_evt_queue = NULL;
//static QueueHandle_t batsw_evt_queue = NULL;

//auto estop0_start = std::chrono::high_resolution_clock::now();
//auto estop1_start = std::chrono::high_resolution_clock::now();
//auto batSW_shutDown = std::chrono::high_resolution_clock::now();
//bool ESTOP0_triggered = false;
//bool ESTOP1_triggered = false;

//bool batSW = false;
//bool batSW_shutDown_flag = false;
int32_t registry_di, registry_do;
int8_t registry_amr_state, registry_bms_level;
uint8_t BMS_percentage = 0;
uint16_t current_NavState = 0;

uint16_t dOut = 0x0000;
bool new_DO = false;

i2c_slave_config i2c_conf = {
	.sda = I2C_SDA, 
	.scl = I2C_SCL, 
	.slaveAddr = 0x0A
};
shoalbot_slave_i2c my_i2c(&i2c_conf);
DI_DO_SPI_config DD_spi_config_0 = {
	.miso = SPI_MISO, 
	.mosi = SPI_MOSI, 
	.sclk = SPI_SCLK, 
	.cs = SPI_CS_2, 
	.bus_init = false,
};
DI_DO_SPI di0(&DD_spi_config_0);
DI_DO_SPI_config DD_spi_config_1 = {
	.miso = SPI_MISO, 
	.mosi = SPI_MOSI, 
	.sclk = SPI_SCLK, 
	.cs = SPI_CS_1, 
	.bus_init = true,
};
DI_DO_SPI di1(&DD_spi_config_1);

DMX::AMRState amr_state = DMX::AMRState::AMR_IDLE;

/*
static void IRAM_ATTR estop_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(estop_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR batsw_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(batsw_evt_queue, &gpio_num, NULL);
}

static void estop_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(estop_evt_queue, &io_num, portMAX_DELAY)) {
            // printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
			if (io_num == DI_0) {
				estop0_start = std::chrono::high_resolution_clock::now();
				ESTOP0_triggered = false;
			}
			if (io_num == DI_1) {
				estop1_start = std::chrono::high_resolution_clock::now();
				ESTOP1_triggered = false;
			}
        }
    }
}

static void batsw_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(batsw_evt_queue, &io_num, portMAX_DELAY)) {
			if (io_num == BAT_SW) {
				if(!batSW) {
					batSW = true;
				}
				else if(batSW) {
					batSW_shutDown = std::chrono::high_resolution_clock::now();
					batSW_shutDown_flag = true;
				}
			}
		}
    }
}
*/
bool ESPdi[8] = {};
uint8_t current_state[6] = {}; // index 0 (MSB) -> index 2 (LSB)
uint8_t di0_data, di1_data;

void set_dOut(void* arg) {
	while(1) {
		if(new_DO) {
			bool d0 = dOut & 0x0001;
			bool d1 = dOut & 0x0002;
			bool d2 = dOut & 0x0004;
			bool d3 = dOut & 0x0008;
			bool d4 = dOut & 0x0010;
			bool d5 = dOut & 0x0020;
			bool d6 = dOut & 0x0040;
			bool d7 = dOut & 0x0080;
			bool d8 = dOut & 0x0100;
			bool d9 = dOut & 0x0200;
			gpio_set_level(DO_0, d0);
			gpio_set_level(DO_1, d1);
			gpio_set_level(DO_2, d2);
			gpio_set_level(DO_3, d3);
			gpio_set_level(DO_4, d4);
			gpio_set_level(DO_5, d5);
			gpio_set_level(DO_6, d6);
			gpio_set_level(DO_7, d7);
			gpio_set_level(DO_8, d8);
			gpio_set_level(DO_9, d9);
			new_DO = false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelete(NULL);
}

void readMap_DO_DI(void* arg) {
	while(1) {
		// FOR DI
		ESPdi[0] = gpio_get_level(DI_0);
		ESPdi[1] = gpio_get_level(DI_1);
		ESPdi[2] = gpio_get_level(DI_12);
		ESPdi[3] = gpio_get_level(DI_13);
		ESPdi[4] = gpio_get_level(DI_14);
		ESPdi[5] = gpio_get_level(DI_15);
		ESPdi[6] = gpio_get_level(DI_16);
		ESPdi[7] = gpio_get_level(DI_18);
		di0_data = di0.read_di(); // printf("spi0: 0x%02X\n", di0_data);
		di1_data = di1.read_di(); // printf("spi1: 0x%02X\n", di1_data);

		// FOR DO
		bool r_d0 = gpio_get_level(DO_0);
		bool r_d1 = gpio_get_level(DO_1);
		bool r_d2 = gpio_get_level(DO_2);
		bool r_d3 = gpio_get_level(DO_3);
		bool r_d4 = gpio_get_level(DO_4);
		bool r_d5 = gpio_get_level(DO_5);
		bool r_d6 = gpio_get_level(DO_6);
		bool r_d7 = gpio_get_level(DO_7);
		bool r_d8 = gpio_get_level(DO_8);
		bool r_d9 = gpio_get_level(DO_9);
		
		// di0_data -> 0b{di9, di8, di7, di6, di5, di4, di3, di2}
		// di1_data -> 0b{di23, di22, di21, di20, di19, di17, di11, di10}
		// etract di17 only from di1_data
		// ESPdi -> 0b{di0, di1, di12, di13, di14, di15, di16, di18}
		// remap di0_data, di1_data, di_esp to current_state

		// current_state[4] -> 0b{do9, do8}
		// current_state[3] -> 0b{do1, do0, do2, do3, do4, do5, do6, do7}
		// current_state[2] -> 0b{di7, di6, di5, di4, di3, di2, di1, di0}
		// current_state[1] -> 0b{di15, di14, di13, di12, di11, di10, di9, di8}
		// current_state[0] -> 0b{di23, di22, di21, di20, di19, di18, di17, di16}

		current_state[5] = 0x01;
		current_state[4] = 0x00 | r_d8 | (r_d9 << 1);
		current_state[3] = 0x00 | r_d0 | (r_d1 << 1) | (r_d2 << 2) | (r_d3 << 3) | (r_d4 << 4) | (r_d5 << 5) | (r_d6 << 6) | (r_d7 << 7);
		current_state[2] = 0x00 | ESPdi[0] | (ESPdi[1] << 1);
		current_state[2] |= (di0_data << 2);
		current_state[1] = 0x00 | (ESPdi[2] << 4) | (ESPdi[3] << 5) | (ESPdi[4] << 6) | (ESPdi[5] << 7);
		current_state[1] |= (di0_data >> 6);
		current_state[0] = 0x00 | (ESPdi[6]) | (ESPdi[7] << 2);
		current_state[0] |= ((di1_data & 0x04) >> 1) | (di1_data & 0xF8);
	/*	printf("DI0: 0x%02X\n", current_DI[0]);
		printf("DI1: 0x%02X\n", current_DI[1]);
		printf("DI2: 0x%02X\n", current_DI[2]);*/
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelete(NULL);
}

void reset_gpio() {
	gpio_reset_pin(DO_0);
	gpio_reset_pin(DO_1);
	gpio_reset_pin(DO_2);
	gpio_reset_pin(DO_3);
	gpio_reset_pin(DO_4);
	gpio_reset_pin(DO_5);
	gpio_reset_pin(DO_6);
	gpio_reset_pin(DO_7);
	gpio_reset_pin(DO_8);
	gpio_reset_pin(DO_9);
	gpio_reset_pin(BMS);
	gpio_reset_pin(PASS_1);
	gpio_reset_pin(PASS_2);
	gpio_reset_pin(DI_0);
	gpio_reset_pin(DI_1);
	gpio_reset_pin(DI_12);
	gpio_reset_pin(DI_13);
	gpio_reset_pin(DI_14);
	gpio_reset_pin(DI_15);
	gpio_reset_pin(DI_16);
	gpio_reset_pin(DI_18);
	gpio_reset_pin(BOOTKEY);
	gpio_set_direction(DO_0, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_3, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_4, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_5, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_6, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_7, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_8, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_9, GPIO_MODE_OUTPUT);
	gpio_set_direction(BMS, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DI_0, GPIO_MODE_INPUT);
	gpio_set_direction(DI_1, GPIO_MODE_INPUT);
	gpio_set_direction(DI_12, GPIO_MODE_INPUT);
	gpio_set_direction(DI_13, GPIO_MODE_INPUT);
	gpio_set_direction(DI_14, GPIO_MODE_INPUT);
	gpio_set_direction(DI_15, GPIO_MODE_INPUT);
	gpio_set_direction(DI_16, GPIO_MODE_INPUT);
	gpio_set_direction(DI_18, GPIO_MODE_INPUT);
	gpio_set_direction(BOOTKEY, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BOOTKEY, GPIO_PULLUP_ONLY);
}

void rs485_task(void *arg) { // DMX task
	while (1) {
		DMX::SetAMRState(DMX::AMRState::AMR_ERROR, 33);;  // Update LED color based on amr_state.
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}

void i2c_task(void* arg) {
	while(1) {
		my_i2c.i2c_read();
		if(my_i2c.get_state() == true) {
			my_i2c.set_state(current_state);
		}
		if(my_i2c.get_do() == true) {
			dOut = my_i2c.return_DO();
			new_DO = true;
			current_NavState = my_i2c.return_NavState();
			BMS_percentage = my_i2c.return_BMS();
			ESP_LOGI("I2C", "current DO State: 0x%04X\n",dOut);
			ESP_LOGI("I2C", "current Nav State: 0x%04X\n",current_NavState);
			ESP_LOGI("I2C", "Battery Percentage: %u\n", BMS_percentage);
		}
	}
	vTaskDelete(NULL);
}

extern "C" void app_main(void) {
	reset_gpio();
	gpio_set_level(BMS, 1);
	gpio_set_level(PASS_1, 1); // Kinco enable 
	gpio_set_level(PASS_2, 1); // N.C.
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set_level(DO_0, 1); // Lidar 24V
	gpio_set_level(DO_1, 1); // LED	24V
	gpio_set_level(DO_2, 0); // Relay K2 to close Auto Charging's 48V loop
	gpio_set_level(DO_3, 1); // Relay K3 for Kinco Power
	di0.begin();
	di1.begin();
//	DMX::Initialize(DMXDirection::DMX_DIR_OUTPUT, 1, 512);
//	xTaskCreate(rs485_task, "rs485_task", 16000, NULL, 1, NULL);
	xTaskCreate(readMap_DO_DI, "readMap_state", 16000, NULL, 5, NULL);
	xTaskCreate(i2c_task, "i2c_task", 16000, NULL, 5, NULL);
	xTaskCreatePinnedToCore(set_dOut, "set_dOut", 16000, NULL, 5, NULL, 1);

// 	while(1) {
// 		uint16_t dOut = my_i2c.i2c_read();
// 		if(my_i2c.get_di() == true) {
// 			my_i2c.set_di(current_DI);
// 		}
// 		if(my_i2c.get_do() == true) {
// 			set_dOut(dOut); // printf("DO: %d\n", dOut);
// 			my_i2c.i2c_send_DO(slave_DO);
// 		}

// /*		auto now = std::chrono::high_resolution_clock::now();
//         auto elapsed_time_estop0 = std::chrono::duration_cast<std::chrono::milliseconds>(now - estop0_start).count();
// 		auto elapsed_time_estop1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - estop1_start).count();

// 		if(batSW_shutDown_flag && gpio_get_level(BATSW) == 1) {
// 			auto elapsed_time_batSW = std::chrono::duration_cast<std::chrono::milliseconds>(now - batSW_shutDown).count();
// 			if(elapsed_time_batSW > 2000) {
// 				batSW = false;
// 				batSW_shutDown_flag = false;
// 			}
// 		}
// 		else {
// 			batSW_shutDown_flag = false;
// 		}
        
//         if (elapsed_time_estop0 > 110) {
// 			printf("ESTOP0 TRIGGERED\n");
//             ESTOP0_triggered = true;
//         }
// 		if (elapsed_time_estop1 > 110) {
// 			printf("ESTOP1 TRIGGERED\n");
// 			ESTOP1_triggered = true;
// 		}

// 		if(ESTOP0_triggered && ESTOP1_triggered) {
// 			// current_DI[0] = 0x01;
// 			printf("EMERGENCY STOP\n");
// 		}
// 		else {
// 			// current_DI[0] = 0x00;
// 		}
// 		if(batSW) {
// 			printf("BATSW ON\n");
// 		}
// 		else if(!batSW) {
// 			printf("BATSW OFF\n");
// 		}
// */
// 	}
}