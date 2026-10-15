#include "board.h"
#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_ctlreq.h"
#include <string.h>

/* Hand-authored BSP, NOT CubeMX output. Pin provenance: output/pcb/pad_connections.csv.
 * RM0390: TIM1/TIM3 ITR1=TIM2; ADC regular EXTSEL=TIM3_TRGO;
 * ADC3 DMA2 stream0 channel2. Scope validation is mandatory before use. */
#define TIMER_APB1_HZ 72000000U
#define TIMER_APB2_HZ 144000000U
#define CARRIER_HZ 1000000U
#define PREBURST_CYCLES 10U
#define RX_CAPACITY 256U
static SPI_HandleTypeDef spi;
static IWDG_HandleTypeDef watchdog;
volatile drive_result board_boot_error;
void _init(void) {}
void _fini(void) {}
static ADC_HandleTypeDef adc;
static DMA_HandleTypeDef dma;
static PCD_HandleTypeDef pcd;
static USBD_HandleTypeDef usb;
static USBD_CDC_HandleTypeDef cdc_storage;
static volatile bool cdc_allocated;
static volatile capture_status acquisition;
static volatile bool usb_online, usb_fault, rx_overflow;
static volatile uint16_t rx_read, rx_write;
static uint8_t rx_ring[RX_CAPACITY], usb_rx[CDC_DATA_FS_MAX_PACKET_SIZE];
static bool adc_initialized;
/* Critical sections cover foreground/USB IRQ ownership of vendor state and queues. */
static uint32_t lock(void) { uint32_t key=__get_PRIMASK(); __disable_irq(); return key; }
static void unlock(uint32_t key) { __set_PRIMASK(key); }
static drive_result hal_result(HAL_StatusTypeDef r) {
    return r==HAL_OK?DRIVE_OK:(r==HAL_TIMEOUT?DRIVE_TIMEOUT:(r==HAL_BUSY?DRIVE_BUSY:DRIVE_IO));
}
static void gpio(GPIO_TypeDef *port, uint32_t pins, uint32_t mode, uint32_t alternate) {
    GPIO_InitTypeDef c={0}; c.Pin=pins; c.Mode=mode; c.Pull=GPIO_NOPULL;
    c.Speed=GPIO_SPEED_FREQ_VERY_HIGH; c.Alternate=alternate; HAL_GPIO_Init(port,&c);
}
static void inhibit(void) {
    /* IO1 high first: inhibit the AFE before changing the carrier mux. */
    GPIOA->BSRR=GPIO_PIN_3|GPIO_PIN_8;
    gpio(GPIOA,GPIO_PIN_3,GPIO_MODE_OUTPUT_PP,0U);
    gpio(GPIOA,GPIO_PIN_8,GPIO_MODE_OUTPUT_PP,0U);
    TIM2->CR1=0U; TIM3->CR1=0U; TIM1->CR1=0U;
    TIM1->SMCR=0U; TIM3->SMCR=0U; TIM1->BDTR=0U;
}
static drive_result stop(void) {
    inhibit();
    drive_result r=DRIVE_OK;
    if (adc_initialized && acquisition!=CAPTURE_IDLE) { r=hal_result(HAL_ADC_Stop_DMA(&adc)); }
    uint32_t key=lock();
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn); HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
    acquisition=CAPTURE_IDLE; unlock(key); return r;
}
static drive_result spi_transfer(const uint8_t command[2], uint8_t response[2], uint32_t timeout) {
    /* 1 ms CS-high/setup gaps conservatively exceed the datasheet minima; foreground only. */
    HAL_Delay(1U); HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_StatusTypeDef r=HAL_SPI_TransmitReceive(&spi,command,response,2U,timeout);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); return hal_result(r);
}
static drive_result capture(uint16_t *buffer, uint16_t count, uint32_t rate, uint8_t pulses) {
    if (count!=DRIVE_SAMPLE_COUNT || (rate!=2400000U && rate!=1200000U) || pulses>20U) { return DRIVE_ARGUMENT; }
    if (acquisition!=CAPTURE_IDLE) { return DRIVE_BUSY; }
    uint32_t sample_ticks=TIMER_APB1_HZ/rate;
    inhibit();
    /* Gate begins at 1 us. Half a sample of tail margin prevents a 2401st trigger. */
    TIM2->PSC=0U; TIM2->ARR=72U+(uint32_t)count*sample_ticks+sample_ticks/2U-1U;
    TIM2->CCR1=72U; TIM2->CCR4=72U*(1U+PREBURST_CYCLES);
    TIM2->CCMR1=TIM_CCMR1_OC1M; /* PWM2 (111), internal gate */
    TIM2->CCMR2=TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2; /* PWM1, active-high inhibit */
    TIM2->CCER=TIM_CCER_CC4E; TIM2->CR2=TIM_CR2_MMS_2; /* OC1REF TRGO */
    TIM2->SMCR=0U; TIM2->DIER=0U; TIM2->EGR=TIM_EGR_UG; TIM2->SR=0U; TIM2->CNT=0U;
    TIM3->PSC=0U; TIM3->ARR=sample_ticks-1U; TIM3->CR2=0U; TIM3->DIER=0U;
    TIM3->EGR=TIM_EGR_UG; TIM3->SR=0U; TIM3->CNT=0U;
    TIM3->CR2=TIM_CR2_MMS_1; /* TRGO update, selected only after UG */
    TIM1->PSC=0U; TIM1->ARR=TIMER_APB2_HZ/CARRIER_HZ-1U; TIM1->CCR1=TIMER_APB2_HZ/CARRIER_HZ/2U;
    /* Include terminal falling edge: N full falling-to-falling drive cycles. */
    TIM1->RCR=PREBURST_CYCLES+(uint32_t)pulses;
    TIM1->CCMR1=TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;
    TIM1->CCER=TIM_CCER_CC1E; TIM1->CR2=TIM_CR2_OIS1; TIM1->DIER=0U;
    TIM1->EGR=TIM_EGR_UG; TIM1->SR=0U; TIM1->CNT=0U;
    TIM1->CR1=TIM_CR1_OPM; TIM1->BDTR=TIM_BDTR_MOE;
    if (pulses!=0U) {
        gpio(GPIOA,GPIO_PIN_3,GPIO_MODE_AF_PP,GPIO_AF1_TIM2);
        gpio(GPIOA,GPIO_PIN_8,GPIO_MODE_AF_PP,GPIO_AF1_TIM1);
    }
    acquisition=CAPTURE_ACTIVE;
    /* HAL accepts uint32_t*; DMA is explicitly halfword aligned, not 32-bit samples. */
    drive_result r=hal_result(HAL_ADC_Start_DMA(&adc,(uint32_t *)(void *)buffer,count));
    if (r!=DRIVE_OK) { acquisition=CAPTURE_ERROR; inhibit(); return r; }
    __HAL_DMA_DISABLE_IT(&dma,DMA_IT_HT);
    if (pulses!=0U) { TIM1->SMCR=TIM_SMCR_TS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2; } /* ITR1, trigger */
    TIM3->SMCR=TIM_SMCR_TS_0|TIM_SMCR_SMS_2|TIM_SMCR_SMS_0; /* ITR1, gated */
    TIM3->CR1=TIM_CR1_CEN;
    TIM2->CR1=TIM_CR1_OPM|TIM_CR1_CEN;
    return DRIVE_OK;
}
static capture_status capture_state(void) { __DMB(); return acquisition; }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *handle) {
    if (handle==&adc && acquisition==CAPTURE_ACTIVE) { __DMB(); acquisition=CAPTURE_DONE; }
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle) {
    if (handle==&adc) { TIM3->CR1=0U; acquisition=CAPTURE_ERROR; }
}
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&dma); }
void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&adc); }
void SysTick_Handler(void) { HAL_IncTick(); }
void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&pcd); }
static bool connected(void) { return usb_online; }
static bool usb_busy(void) {
    uint32_t key=lock(); bool busy=cdc_allocated && cdc_storage.TxState!=0U; unlock(key); return busy;
}
static drive_result usb_send(const uint8_t *data, uint16_t length) {
    uint32_t key=lock(); drive_result r=DRIVE_DISCONNECTED;
    if (usb_online && cdc_allocated && usb.dev_state==USBD_STATE_CONFIGURED) {
        if (cdc_storage.TxState!=0U) { r=DRIVE_BUSY; }
        else {
            uint8_t s=USBD_CDC_SetTxBuffer(&usb,(uint8_t *)(uintptr_t)data,length);
            if (s==USBD_OK) { s=USBD_CDC_TransmitPacket(&usb); }
            r=s==USBD_OK?DRIVE_OK:(s==USBD_BUSY?DRIVE_BUSY:DRIVE_IO);
        }
    }
    unlock(key); return r;
}
drive_port board_port={spi_transfer,HAL_Delay,HAL_GetTick,capture,capture_state,stop,connected,usb_busy,usb_send,0U};
void *usb_static_alloc(uint32_t size) {
    if (cdc_allocated || size>sizeof(cdc_storage)) { usb_fault=true; return NULL; }
    cdc_allocated=true; return &cdc_storage;
}
void usb_static_free(void *p) {
    if (p==&cdc_storage) { cdc_allocated=false; }
    else { usb_fault=true; }
}
static int8_t cdc_init(void) {
    uint8_t r=USBD_CDC_SetRxBuffer(&usb,usb_rx);
    if (r==USBD_OK) { usb_online=true; }
    return (int8_t)r;
}
static int8_t cdc_deinit(void) { usb_online=false; rx_read=0U; rx_write=0U; return (int8_t)USBD_OK; }
static int8_t cdc_control(uint8_t cmd, uint8_t *data, uint16_t length) {
    static uint8_t line[7]={0U,0xc2U,1U,0U,0U,0U,8U};
    if (cmd==CDC_SET_LINE_CODING || cmd==CDC_GET_LINE_CODING) {
        if (length!=7U) { return (int8_t)USBD_FAIL; }
        if (cmd==CDC_SET_LINE_CODING) { memcpy(line,data,7U); }
        else { memcpy(data,line,7U); }
    }
    return (int8_t)USBD_OK;
}
static int8_t cdc_receive(uint8_t *data, uint32_t *length) {
    for (uint32_t i=0; i<*length; ++i) {
        uint16_t next=(uint16_t)(((uint32_t)rx_write+1U)%RX_CAPACITY);
        if (next==rx_read) { rx_overflow=true; break; }
        rx_ring[rx_write]=data[i]; __DMB(); rx_write=next;
    }
    uint8_t r=USBD_CDC_ReceivePacket(&usb);
    if (r!=USBD_OK) { usb_fault=true; }
    return (int8_t)r;
}
static int8_t cdc_complete(uint8_t *data, uint32_t *length, uint8_t ep) {
    (void)data; (void)length; (void)ep; return (int8_t)USBD_OK;
}
static USBD_CDC_ItfTypeDef cdc_interface={cdc_init,cdc_deinit,cdc_control,cdc_receive,cdc_complete};
/* ST demonstration VID/PID, development only; assign a permitted ID before distribution. */
static uint8_t device_descriptor[18]={18U,1U,0U,2U,2U,2U,0U,64U,0x83U,0x04U,0x40U,0x57U,0U,1U,1U,2U,3U,1U};
static uint8_t language_descriptor[4]={4U,3U,9U,4U};
static uint8_t string_descriptor[128];
static uint8_t *desc_device(USBD_SpeedTypeDef s,uint16_t *n) { (void)s; *n=18U; return device_descriptor; }
static uint8_t *desc_lang(USBD_SpeedTypeDef s,uint16_t *n) { (void)s; *n=4U; return language_descriptor; }
static uint8_t *desc_string(const char *s,uint16_t *n) { USBD_GetString((uint8_t *)(uintptr_t)s,string_descriptor,n); return string_descriptor; }
static uint8_t *desc_maker(USBD_SpeedTypeDef s,uint16_t *n) { (void)s; return desc_string("UltraTac development",n); }
static uint8_t *desc_product(USBD_SpeedTypeDef s,uint16_t *n) { (void)s; return desc_string("Provisional F446 envelope",n); }
static uint8_t *desc_serial(USBD_SpeedTypeDef s,uint16_t *n) {
    (void)s; char serial[25]; static const char hex[]="0123456789ABCDEF";
    const uint32_t *uid=(const uint32_t *)UID_BASE;
    for (uint32_t w=0U; w<3U; ++w) {
        for (uint32_t b=0U; b<8U; ++b) { serial[w*8U+b]=hex[(uid[w]>>(28U-4U*b))&15U]; }
    }
    serial[24]='\0'; return desc_string(serial,n);
}
static USBD_DescriptorsTypeDef descriptors={desc_device,desc_lang,desc_maker,desc_product,desc_serial,desc_product,desc_product};
static USBD_StatusTypeDef usb_result(HAL_StatusTypeDef s) {
    if (s!=HAL_OK) { usb_fault=true; }
    return s==HAL_OK?USBD_OK:(s==HAL_BUSY?USBD_BUSY:USBD_FAIL);
}
static void usb_check(USBD_StatusTypeDef s) { if (s!=USBD_OK) { usb_fault=true; } }
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *d) {
    pcd.Instance=USB_OTG_FS; pcd.Init.dev_endpoints=4U; pcd.Init.speed=PCD_SPEED_FULL;
    pcd.Init.dma_enable=DISABLE; pcd.Init.phy_itface=PCD_PHY_EMBEDDED;
    pcd.Init.Sof_enable=DISABLE; pcd.Init.low_power_enable=DISABLE;
    pcd.Init.lpm_enable=DISABLE; pcd.Init.vbus_sensing_enable=DISABLE;
    pcd.Init.use_dedicated_ep1=DISABLE; pcd.pData=d; d->pData=&pcd;
    HAL_StatusTypeDef s=HAL_PCD_Init(&pcd);
    if (s==HAL_OK) { s=HAL_PCDEx_SetRxFiFo(&pcd,128U); }
    if (s==HAL_OK) { s=HAL_PCDEx_SetTxFiFo(&pcd,0U,64U); }
    if (s==HAL_OK) { s=HAL_PCDEx_SetTxFiFo(&pcd,1U,64U); }
    if (s==HAL_OK) { s=HAL_PCDEx_SetTxFiFo(&pcd,2U,64U); }
    return usb_result(s);
}
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *d) { (void)d; return usb_result(HAL_PCD_DeInit(&pcd)); }
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *d) { (void)d; return usb_result(HAL_PCD_Start(&pcd)); }
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *d) { (void)d; return usb_result(HAL_PCD_Stop(&pcd)); }
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *d,uint8_t a,uint8_t t,uint16_t m) { (void)d; return usb_result(HAL_PCD_EP_Open(&pcd,a,m,t)); }
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return usb_result(HAL_PCD_EP_Close(&pcd,a)); }
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return usb_result(HAL_PCD_EP_Flush(&pcd,a)); }
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return usb_result(HAL_PCD_EP_SetStall(&pcd,a)); }
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return usb_result(HAL_PCD_EP_ClrStall(&pcd,a)); }
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return usb_result(HAL_PCD_SetAddress(&pcd,a)); }
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *d,uint8_t a,uint8_t *p,uint32_t n) { (void)d; return usb_result(HAL_PCD_EP_Transmit(&pcd,a,p,n)); }
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *d,uint8_t a,uint8_t *p,uint32_t n) { (void)d; return usb_result(HAL_PCD_EP_Receive(&pcd,a,p,n)); }
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return (uint8_t)(((a&0x80U)!=0U)?pcd.IN_ep[a&0x7fU].is_stall:pcd.OUT_ep[a&0x7fU].is_stall); }
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *d,uint8_t a) { (void)d; return HAL_PCD_EP_GetRxCount(&pcd,a); }
void USBD_LL_Delay(uint32_t ms) { HAL_Delay(ms); }
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *p) { usb_check(USBD_LL_SetupStage(&usb,(uint8_t *)(void *)p->Setup)); }
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *p,uint8_t ep) { usb_check(USBD_LL_DataOutStage(&usb,ep,p->OUT_ep[ep].xfer_buff)); }
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *p,uint8_t ep) { usb_check(USBD_LL_DataInStage(&usb,ep,p->IN_ep[ep].xfer_buff)); }
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *p) { (void)p; if (usb_online) { usb_fault=true; } usb_online=false; usb_check(USBD_LL_SetSpeed(&usb,USBD_SPEED_FULL)); usb_check(USBD_LL_Reset(&usb)); }
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *p) { (void)p; usb_check(USBD_LL_Suspend(&usb)); usb_fault=true; }
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *p) { (void)p; usb_check(USBD_LL_Resume(&usb)); }
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *p) { (void)p; usb_check(USBD_LL_DevConnected(&usb)); }
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *p) { (void)p; usb_check(USBD_LL_DevDisconnected(&usb)); usb_fault=true; }

drive_result board_init(void) {
    board_port.reset_cause=RCC->CSR; __HAL_RCC_CLEAR_RESET_FLAGS();
    drive_result r=hal_result(HAL_Init()); if (r!=DRIVE_OK) { return r; }
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE(); __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
    inhibit(); GPIOC->BSRR=GPIO_PIN_4; gpio(GPIOC,GPIO_PIN_4,GPIO_MODE_OUTPUT_PP,0U);
    __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitTypeDef osc={0}; osc.OscillatorType=RCC_OSCILLATORTYPE_HSE;
    osc.HSEState=RCC_HSE_ON; osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM=25U; osc.PLL.PLLN=288U; osc.PLL.PLLP=RCC_PLLP_DIV2; osc.PLL.PLLQ=6U; osc.PLL.PLLR=2U;
    r=hal_result(HAL_RCC_OscConfig(&osc)); if (r!=DRIVE_OK) { return r; }
    RCC_ClkInitTypeDef clocks={0}; clocks.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clocks.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider=RCC_HCLK_DIV4; clocks.APB2CLKDivider=RCC_HCLK_DIV2;
    r=hal_result(HAL_RCC_ClockConfig(&clocks,FLASH_LATENCY_4)); if (r!=DRIVE_OK) { return r; }
    gpio(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2,GPIO_MODE_ANALOG,0U);
    gpio(GPIOC,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_MODE_ANALOG,0U);
    gpio(GPIOC,GPIO_PIN_5,GPIO_MODE_INPUT,0U); gpio(GPIOB,GPIO_PIN_0,GPIO_MODE_INPUT,0U);
    __HAL_RCC_SPI1_CLK_ENABLE(); gpio(GPIOA,GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_MODE_AF_PP,GPIO_AF5_SPI1);
    spi.Instance=SPI1; spi.Init.Mode=SPI_MODE_MASTER; spi.Init.Direction=SPI_DIRECTION_2LINES;
    spi.Init.DataSize=SPI_DATASIZE_8BIT; spi.Init.CLKPolarity=SPI_POLARITY_LOW; spi.Init.CLKPhase=SPI_PHASE_2EDGE;
    spi.Init.NSS=SPI_NSS_SOFT; spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_64;
    spi.Init.FirstBit=SPI_FIRSTBIT_MSB; spi.Init.TIMode=SPI_TIMODE_DISABLE; spi.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    r=hal_result(HAL_SPI_Init(&spi)); if (r!=DRIVE_OK) { return r; }
    __HAL_RCC_ADC3_CLK_ENABLE(); __HAL_RCC_DMA2_CLK_ENABLE();
    dma.Instance=DMA2_Stream0; dma.Init.Channel=DMA_CHANNEL_2; dma.Init.Direction=DMA_PERIPH_TO_MEMORY;
    dma.Init.PeriphInc=DMA_PINC_DISABLE; dma.Init.MemInc=DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD; dma.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
    dma.Init.Mode=DMA_NORMAL; dma.Init.Priority=DMA_PRIORITY_VERY_HIGH; dma.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    r=hal_result(HAL_DMA_Init(&dma)); if (r!=DRIVE_OK) { return r; }
    adc.Instance=ADC3; adc.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV2; adc.Init.Resolution=ADC_RESOLUTION_12B;
    adc.Init.ScanConvMode=DISABLE; adc.Init.ContinuousConvMode=DISABLE; adc.Init.DiscontinuousConvMode=DISABLE;
    adc.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING; adc.Init.ExternalTrigConv=ADC_EXTERNALTRIGCONV_T3_TRGO;
    adc.Init.DataAlign=ADC_DATAALIGN_RIGHT; adc.Init.NbrOfConversion=1U; adc.Init.DMAContinuousRequests=ENABLE;
    adc.Init.EOCSelection=ADC_EOC_SINGLE_CONV; __HAL_LINKDMA(&adc,DMA_Handle,dma);
    r=hal_result(HAL_ADC_Init(&adc)); if (r!=DRIVE_OK) { return r; }
    ADC_ChannelConfTypeDef ch={0}; ch.Channel=ADC_CHANNEL_0; ch.Rank=1U; ch.SamplingTime=ADC_SAMPLETIME_3CYCLES;
    r=hal_result(HAL_ADC_ConfigChannel(&adc,&ch)); if (r!=DRIVE_OK) { return r; }
    adc_initialized=true;
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn,1U,0U); HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn,1U,0U); HAL_NVIC_EnableIRQ(ADC_IRQn);
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE(); gpio(GPIOA,GPIO_PIN_11|GPIO_PIN_12,GPIO_MODE_AF_PP,GPIO_AF10_OTG_FS);
    HAL_NVIC_SetPriority(OTG_FS_IRQn,2U,0U); HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    if (USBD_Init(&usb,&descriptors,0U)!=USBD_OK || USBD_RegisterClass(&usb,&USBD_CDC)!=USBD_OK ||
        USBD_CDC_RegisterInterface(&usb,&cdc_interface)!=USBD_OK || USBD_Start(&usb)!=USBD_OK) { return DRIVE_IO; }
    return DRIVE_OK;
}
void board_poll(void) {
    if (usb_fault || rx_overflow) {
        /* Drain hardware ownership BEFORE allowing the app to recycle TX storage. */
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        usb_fault=false; /* Clear recovery reason; subsequent LL errors must remain visible. */
        drive_result r=stop();
        USBD_StatusTypeDef s=USBD_LL_Stop(&usb);
        if (s!=USBD_OK) { NVIC_SystemReset(); }
        s=USBD_Stop(&usb);
        if (r!=DRIVE_OK || s!=USBD_OK || usb_fault) { NVIC_SystemReset(); }
        /* Some middleware APIs discard LL errors; inspect our sticky adapter flag. */
        usb_online=false; usb_fault=false; rx_overflow=false; rx_read=0U; rx_write=0U;
        drive_process(); /* Disconnect cancels streaming, including across re-enumeration. */
        s=USBD_DeInit(&usb);
        if (s!=USBD_OK || usb_fault) { NVIC_SystemReset(); }
        HAL_Delay(20U);
        if (USBD_Init(&usb,&descriptors,0U)!=USBD_OK || USBD_RegisterClass(&usb,&USBD_CDC)!=USBD_OK ||
            USBD_CDC_RegisterInterface(&usb,&cdc_interface)!=USBD_OK || USBD_Start(&usb)!=USBD_OK) { NVIC_SystemReset(); }
        HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn); HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
    uint8_t fragment[64]; size_t used=0U; uint32_t key=lock();
    while (rx_read!=rx_write && used<sizeof(fragment)) {
        fragment[used++]=rx_ring[rx_read]; rx_read=(uint16_t)(((uint32_t)rx_read+1U)%RX_CAPACITY);
    }
    unlock(key);
    if (used!=0U) { drive_receive(fragment,used); }
}
/* Fault handlers inhibit outputs; watchdog/reset policy does not turn off the boost rail. */
void HardFault_Handler(void) { inhibit(); NVIC_SystemReset(); }
void MemManage_Handler(void) { inhibit(); NVIC_SystemReset(); }
void BusFault_Handler(void) { inhibit(); NVIC_SystemReset(); }
void UsageFault_Handler(void) { inhibit(); NVIC_SystemReset(); }
int main(void) {
    drive_result r=board_init();
    board_boot_error=r;
    if (r!=DRIVE_OK) { inhibit(); for (;;) { __WFI(); } }
    r=drive_init(&board_port);
    /* AFE failure remains observable over USB and can be explicitly recovered. */
    board_boot_error=r;
    watchdog.Instance=IWDG; watchdog.Init.Prescaler=IWDG_PRESCALER_64; watchdog.Init.Reload=1999U;
    if (HAL_IWDG_Init(&watchdog)!=HAL_OK) { inhibit(); NVIC_SystemReset(); }
    for (;;) {
        board_poll(); drive_process();
        if (HAL_IWDG_Refresh(&watchdog)!=HAL_OK) { inhibit(); NVIC_SystemReset(); }
    }
}
