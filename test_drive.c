#include "drive.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t regs[64];
static const char *golden_path;
static uint32_t now, captures, sends;
static bool online, busy, bad_parity, previous_error, mismatch;
static drive_result spi_error, stop_error;
static capture_status hardware;
static const uint8_t *owned;
static uint16_t owned_length;
static uint8_t saved[DRIVE_FRAME_SIZE];
static uint8_t odd(uint8_t a,uint8_t b) {
    unsigned ones=0U;
    for (unsigned i=0U;i<8U;++i) { ones+=((unsigned)a>>i)&1U; ones+=((unsigned)b>>i)&1U; }
    return (uint8_t)(ones&1U);
}
static drive_result spi(const uint8_t command[2],uint8_t response[2],uint32_t timeout) {
    assert(timeout>0U); assert(odd(command[0],command[1])==1U);
    if (spi_error!=DRIVE_OK) { return spi_error; }
    uint8_t addr=(uint8_t)((command[0]>>1)&0x3fU);
    if ((command[0]&0x80U)==0U) { regs[addr]=mismatch?0U:command[1]; }
    response[0]=previous_error?0x80U:0U; response[1]=regs[addr];
    response[0]|=(uint8_t)(odd(response[0],response[1])^1U);
    if (bad_parity) { response[0]^=1U; }
    return DRIVE_OK;
}
static void delay(uint32_t ms) { now+=ms; }
static uint32_t millis(void) { return now; }
static drive_result capture(uint16_t *data,uint16_t count,uint32_t rate,uint8_t pulses) {
    assert(!busy); assert(hardware==CAPTURE_IDLE); assert(count==2400U);
    assert(rate==2400000U || rate==1200000U); assert(pulses<=20U);
    for (uint32_t i=0U;i<count;++i) { data[i]=(uint16_t)((i*17U)&4095U); }
    ++captures; hardware=CAPTURE_ACTIVE; return DRIVE_OK;
}
static capture_status capture_state(void) { return hardware; }
static drive_result stop(void) { hardware=CAPTURE_IDLE; return stop_error; }
static bool connected(void) { return online; }
static bool usb_busy(void) { return busy; }
static drive_result send(const uint8_t *data,uint16_t length) {
    assert(!busy); assert(length<=sizeof(saved));
    memcpy(saved,data,length); owned=data; owned_length=length; busy=true; ++sends; return DRIVE_OK;
}
static const drive_port port={spi,delay,millis,capture,capture_state,stop,connected,usb_busy,send,0x1234U};
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void put32(uint8_t *p,uint32_t v) { for (uint32_t i=0;i<4U;++i) { p[i]=(uint8_t)(v>>(8U*i)); } }
static void setup(void) {
    memset(regs,0,sizeof(regs)); regs[0x1d]=0xb9U; regs[0x1e]=2U;
    now=0U; captures=0U; sends=0U; online=true; busy=false; bad_parity=false;
    previous_error=false; mismatch=false; spi_error=DRIVE_OK; stop_error=DRIVE_OK;
    hardware=CAPTURE_IDLE; owned=NULL; owned_length=0U;
    assert(drive_init(&port)==DRIVE_OK); assert(drive_get_state()==DRIVE_IDLE);
    drive_process(); assert(captures==0U);
}
static void finish_usb(void) {
    assert(owned!=NULL); assert(memcmp(owned,saved,owned_length)==0);
    busy=false; drive_process();
}
static void command(uint8_t opcode,uint32_t id,uint32_t arg) {
    uint8_t bytes[20]={ 'E','F','C','1',0U,0U,0U,0U };
    bytes[4]=opcode; put32(bytes+8,id); put32(bytes+12,arg); put32(bytes+16,drive_crc32(bytes,16U));
    /* Exercise arbitrary CDC fragmentation, including the CRC boundary. */
    drive_receive(bytes,3U); drive_receive(bytes+3,14U); drive_receive(bytes+17,3U);
}
static void test_capture(void) {
    setup(); command(3U,42U,0U); assert(captures==1U); drive_process();
    assert(saved[5]==2U && u32(saved+12)==42U); finish_usb();
    hardware=CAPTURE_DONE; drive_process(); assert(saved[5]==3U);
    assert(owned_length==DRIVE_FRAME_SIZE); assert(u32(saved+8)==DRIVE_FRAME_SIZE);
    assert(u32(saved+20)==2400000U); assert(u32(saved+owned_length-4U)==drive_crc32(saved,owned_length-4U));
    for (size_t i=0;i<DRIVE_SAMPLE_COUNT;++i) {
        uint16_t sample=(uint16_t)((uint16_t)saved[48U+2U*i]|((uint16_t)saved[49U+2U*i]<<8));
        assert(sample==(uint16_t)((i*17U)&4095U));
    }
    if (golden_path!=NULL) {
        FILE *f=fopen(golden_path,"wb"); assert(f!=NULL);
        assert(fwrite(saved,1U,owned_length,f)==owned_length); assert(fclose(f)==0);
    }
    /* STOP may cancel future work but must never rewrite an in-flight frame. */
    command(5U,43U,0U); drive_process(); assert(memcmp(owned,saved,owned_length)==0);
    finish_usb(); assert(saved[5]==2U); finish_usb(); assert(drive_get_state()==DRIVE_IDLE);
}
static void test_faults(void) {
    setup(); command(8U,1U,0U); drive_process(); finish_usb(); hardware=CAPTURE_DONE; drive_process();
    assert(saved[26]==0U && (saved[27]&16U)!=0U && u32(saved+32)==0U); finish_usb();
    setup(); bad_parity=true; command(3U,1U,0U); assert(drive_get_error()==DRIVE_PARITY && captures==0U);
    bad_parity=false; drive_process(); finish_usb(); command(6U,2U,0U);
    assert(drive_get_state()==DRIVE_IDLE && captures==0U); /* Recovery stays disarmed. */
    setup(); previous_error=true; command(3U,1U,0U); assert(drive_get_error()==DRIVE_PARITY);
    setup(); spi_error=DRIVE_TIMEOUT; command(3U,1U,0U); assert(drive_get_error()==DRIVE_TIMEOUT);
    setup(); mismatch=true; command(3U,1U,0U); assert(drive_get_error()==DRIVE_MISMATCH);
    setup(); command(3U,1U,0U); now+=11U; drive_process(); assert(drive_get_error()==DRIVE_TIMEOUT);
    setup(); command(3U,1U,0U); hardware=CAPTURE_ERROR; drive_process(); assert(drive_get_error()==DRIVE_ADC);
    setup(); command(3U,1U,0U); regs[0x1c]=4U; hardware=CAPTURE_DONE;
    drive_process(); assert(drive_get_error()==DRIVE_DEVICE);
    setup(); command(3U,1U,0U); drive_process(); now+=1001U; drive_process();
    assert(drive_get_error()==DRIVE_TIMEOUT); assert(memcmp(owned,saved,owned_length)==0);
    setup(); command(3U,1U,0U); online=false; drive_process(); assert(hardware==CAPTURE_IDLE);
    online=true; drive_process(); assert(captures==1U && drive_get_state()==DRIVE_IDLE);
    setup(); regs[0x1d]=0U; assert(drive_init(&port)==DRIVE_DEVICE);
    setup(); regs[0x1c]=1U; assert(drive_init(&port)==DRIVE_DEVICE);
    setup(); stop_error=DRIVE_IO; command(5U,1U,0U); assert(drive_get_error()==DRIVE_IO);
}
static void test_parser_stream(void) {
    setup(); uint8_t noise[37]; memset(noise,0xa5,sizeof(noise)); drive_receive(noise,sizeof(noise));
    command(2U,1U,20U); drive_process(); assert(saved[6]==DRIVE_OK && saved[26]==20U && u32(saved+20)==1200000U); finish_usb();
    command(2U,2U,0U); drive_process(); assert(saved[6]==DRIVE_ARGUMENT); finish_usb();
    command(4U,3U,20U); drive_process(); finish_usb();
    now+=20U; drive_process(); assert(captures==1U);
    hardware=CAPTURE_DONE; drive_process(); assert(saved[5]==3U);
    now+=100U; drive_process(); assert(captures==1U); /* Backpressure: no buffer reuse. */
    finish_usb(); now+=20U; drive_process(); assert(captures==2U);
    command(5U,4U,0U); assert(hardware==CAPTURE_IDLE); drive_process();
    finish_usb(); now+=100U; drive_process(); assert(captures==2U);
    setup(); now=UINT32_MAX-10U; command(4U,1U,20U); drive_process(); finish_usb();
    now+=20U; drive_process(); assert(captures==1U); /* Tick wrap. */
    setup(); uint8_t invalid[20]={'E','F','C','1',3U};
    drive_receive(invalid,sizeof(invalid)); assert(captures==0U);
    command(3U,1U,0U); assert(captures==1U);
    for (uint32_t i=0U;i<10U;++i) { command(7U,i,0U); }
    command(5U,20U,0U); assert(hardware==CAPTURE_IDLE); /* Saturated queue STOP. */
}
int main(int argc,char **argv) {
    golden_path=argc>1?argv[1]:NULL;
    assert(drive_crc32((const uint8_t *)"123456789",9U)==UINT32_C(0xcbf43926));
    assert(drive_init(NULL)==DRIVE_ARGUMENT);
    test_capture(); test_faults(); test_parser_stream();
    puts("PASS: capture, CRC/fragments, parity/readback, faults, USB ownership, backpressure, STOP, tick wrap");
    return 0;
}
