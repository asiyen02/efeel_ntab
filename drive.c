#include "TUSS4470.h"

uint16_t init_check_buffer[13] = {0};
uint16_t write_back[16] = {0};
uint32_t ADC_raw_data[3] = {0};

// Define 16-bit ADC sampling data buffer
uint16_t ADC_data1[SAMPLE_COUNT];

// Each two ADC samples occupy 3 bytes, plus 4 bytes for header and tail
uint8_t UART_Buffer[SAMPLE_COUNT * 3 / 2 + 4];

uint8_t pulse_flag = 0;   // Initial pulse state set to 0
uint8_t buffer_index = 0; // Current buffer index

static uint8_t calculateParity(uint8_t txbuffer[]);

void TUSS4470_init()
{
    HAL_Delay(100);
    TUSS4470_Write_Register(BPF_CONFIG_1, 0x5F, &write_back[0]);
    HAL_Delay(1);
    TUSS4470_Write_Register(BPF_CONFIG_2, 0x00, &write_back[1]);
    HAL_Delay(1);
    TUSS4470_Write_Register(DEV_CTRL_1, 0xB7, &write_back[2]);
    HAL_Delay(1);
    TUSS4470_Write_Register(DEV_CTRL_2, 0x42, &write_back[3]);
    HAL_Delay(1);
    TUSS4470_Write_Register(DEV_CTRL_3, 0x01, &write_back[4]);
    HAL_Delay(1);
    TUSS4470_Write_Register(VDRV_CTRL, 0x3F, &write_back[5]);
    HAL_Delay(1);
    TUSS4470_Write_Register(ECHO_INT_CONFIG, 0x00, &write_back[6]);
    HAL_Delay(1);
    TUSS4470_Write_Register(ZC_CONFIG, 0x00, &write_back[7]);
    HAL_Delay(1);
    TUSS4470_Write_Register(BURST_PULSE, 0x0A, &write_back[8]);
    HAL_Delay(1);
    TUSS4470_Write_Register(TOF_CONFIG, 0x00, &write_back[9]);
    HAL_Delay(1);

    // Read back registers to verify configuration
    TUSS4470_Read_Register(BPF_CONFIG_1, &init_check_buffer[0]);
    HAL_Delay(1);
    TUSS4470_Read_Register(BPF_CONFIG_2, &init_check_buffer[1]);
    HAL_Delay(1);
    TUSS4470_Read_Register(DEV_CTRL_1, &init_check_buffer[2]);
    HAL_Delay(1);
    TUSS4470_Read_Register(DEV_CTRL_2, &init_check_buffer[3]);
    HAL_Delay(1);
    TUSS4470_Read_Register(DEV_CTRL_3, &init_check_buffer[4]);
    HAL_Delay(1);
    TUSS4470_Read_Register(VDRV_CTRL, &init_check_buffer[5]);
    HAL_Delay(1);
    TUSS4470_Read_Register(ECHO_INT_CONFIG, &init_check_buffer[6]);
    HAL_Delay(1);
    TUSS4470_Read_Register(ZC_CONFIG, &init_check_buffer[7]);
    HAL_Delay(1);
    TUSS4470_Read_Register(BURST_PULSE, &init_check_buffer[8]);
    HAL_Delay(1);
    TUSS4470_Read_Register(TOF_CONFIG, &init_check_buffer[9]);
    HAL_Delay(1);
    TUSS4470_Read_Register(DEV_STAT, &init_check_buffer[10]);
    HAL_Delay(1);
    TUSS4470_Read_Register(DEVICE_ID, &init_check_buffer[11]);
    HAL_Delay(1);
    TUSS4470_Read_Register(REV_ID, &init_check_buffer[12]);
    HAL_Delay(1);
};

void TUSS4470_Read_Data()
{
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ADC_data1, SAMPLE_COUNT);
    __asm volatile("nop");
}

void TUSS4470_TransmitData()
{
    // Packet header
    UART_Buffer[0] = 0xAA;
    UART_Buffer[1] = 0x55;

    // Pack ADC data into UART buffer
    for (int i = 0; i < SAMPLE_COUNT; i += 2)
    {
        UART_Buffer[i / 2 * 3 + 2] = (uint8_t)(ADC_data1[i] & 0xFF);                                             // Lower 8 bits of first sample
        UART_Buffer[i / 2 * 3 + 3] = (uint8_t)(((ADC_data1[i] >> 8) & 0x0F) | ((ADC_data1[i + 1] & 0x0F) << 4)); // High 4 bits of first + Low 4 bits of second
        UART_Buffer[i / 2 * 3 + 4] = (uint8_t)((ADC_data1[i + 1] >> 4) & 0xFF);                                  // High 8 bits of second sample
    }

    // Packet tail
    UART_Buffer[SAMPLE_COUNT * 3 / 2 + 2] = 0x55;
    UART_Buffer[SAMPLE_COUNT * 3 / 2 + 3] = 0xAA;

    // Transmit via USB CDC using DMA
    CDC_Transmit_FS(UART_Buffer, SAMPLE_COUNT * 3 / 2 + 4);
    __asm volatile("nop");
}

void TUSS4470_GeneratePulses()
{
    // Not implemented
}

uint16_t receivedData;

void TUSS4470_Read_Register(uint8_t regAddr, uint16_t *data)
{
    uint8_t txbuffer[2] = {0}; // SPI transmit frame
    uint8_t rxbuffer[2] = {0}; // SPI receive frame

    // Build SPI command frame
    txbuffer[0] = 0x80 + ((regAddr & 0x3F) << 1); // MSB: RW bit + address bits
    txbuffer[1] = 0x00;                           // LSB: always 0
    txbuffer[0] |= calculateParity(txbuffer);     // Set parity bit (bit 0) in MSB

    // Start SPI communication
    TUSS4470_CSN_LOW();
    HAL_SPI_TransmitReceive(&hspi1, txbuffer, rxbuffer, 2, HAL_MAX_DELAY);
    TUSS4470_CSN_HIGH(); // End SPI communication

    receivedData = (rxbuffer[0] << 8) | rxbuffer[1];
    *data = receivedData; // Combine MSB and LSB
}

void TUSS4470_Write_Register(uint8_t regAddr, uint8_t data, uint16_t *write_back)
{
    uint8_t txbuffer[2] = {0}; // SPI transmit frame
    uint8_t rxbuffer[2] = {0}; // SPI receive frame

    // Build SPI command frame
    txbuffer[0] = (regAddr & 0x3F) << 1;      // MSB: RW bit (0) + address
    txbuffer[1] = data;                       // LSB: data
    txbuffer[0] |= calculateParity(txbuffer); // Set parity bit in MSB

    // Start SPI communication
    TUSS4470_CSN_LOW();
    HAL_SPI_TransmitReceive(&hspi1, txbuffer, rxbuffer, 2, HAL_MAX_DELAY);
    TUSS4470_CSN_HIGH(); // End SPI communication

    *write_back = (rxbuffer[0] << 8) | rxbuffer[1]; // Combine MSB and LSB
}

static uint8_t calculateParity(uint8_t txbuffer[])
{
    uint8_t parity = 0;

    // Calculate parity across the two-byte buffer
    for (int i = 0; i < 2; i++)
    {
        uint8_t byte = txbuffer[i];
        while (byte)
        {
            parity ^= (byte & 1);
            byte >>= 1;
        }
    }

    // Return even parity (invert for odd parity)
    return parity ^ 1;
}
