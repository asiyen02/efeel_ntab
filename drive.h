#ifndef TUSS4470_H_
#define TUSS4470_H_

#include "my_main.h"
#include <stdint.h>
#define TUSS4470_CSN_LOW() HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, GPIO_PIN_RESET)
#define TUSS4470_CSN_HIGH() HAL_GPIO_WritePin(SPI1_NCS_GPIO_Port, SPI1_NCS_Pin, GPIO_PIN_SET)
#define TUSS4470_IO1_LOW() HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_RESET)
#define TUSS4470_IO1_HIGH() HAL_GPIO_WritePin(IO1_GPIO_Port, IO1_Pin, GPIO_PIN_SET)

#define BPF_CONFIG_1 0x10
#define BPF_CONFIG_2 0x11
#define DEV_CTRL_1 0x12
#define DEV_CTRL_2 0x13
#define DEV_CTRL_3 0x14
#define VDRV_CTRL 0x16
#define ECHO_INT_CONFIG 0x17
#define ZC_CONFIG 0x18
#define BURST_PULSE 0x1A
#define TOF_CONFIG 0x1B
#define DEV_STAT 0x1C
#define DEVICE_ID 0x1D
#define REV_ID 0x1E

void TUSS4470_init(void);
void TUSS4470_Read_Data(void);
void TUSS4470_Read_Register(uint8_t regAddr, uint16_t *data);
void TUSS4470_Write_Register(uint8_t regAddr, uint8_t data, uint16_t *write_back);
void TUSS4470_GeneratePulses(void);
void TUSS4470_TransmitData(void);

#define SAMPLE_COUNT 2400

extern uint16_t init_check_buffer[13];
extern uint32_t ADC_raw_data[3];
extern uint16_t ADC_data1[SAMPLE_COUNT];

extern uint16_t write_back[16];
extern uint8_t pulse_flag;
#endif
