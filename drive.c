#include "drive.h"
#include <string.h>

/* TUSS4470 datasheet SLDS251A: register map and SPI odd parity over all 16 bits. */
enum { REG_BPF1=0x10, REG_BPF2, REG_CTRL1, REG_CTRL2, REG_CTRL3,
       REG_VDRV=0x16, REG_ECHO, REG_ZC, REG_BURST=0x1a, REG_TOF,
       REG_STATUS, REG_ID, REG_REV };
enum { CMD_INFO=1, CMD_CONFIG, CMD_CAPTURE, CMD_STREAM, CMD_STOP, CMD_RECOVER, CMD_STATUS, CMD_BASELINE };
enum { FRAME_STATUS=1, FRAME_ACK, FRAME_WAVE, FRAME_ERROR };
#define REPLY_SIZE (DRIVE_HEADER_SIZE + 4U)
#define REPLY_COUNT 4U
#define SPI_TIMEOUT_MS 10U
#define CAPTURE_TIMEOUT_MS 10U
#define TRANSPORT_TIMEOUT_MS 1000U
#define RATE_FAST 2400000U
#define RATE_DIAGNOSTIC 1200000U
#define CARRIER_HZ 1000000U
static const drive_port *io;
static drive_status state;
static drive_result error;
static bool streaming, pending, configured, was_connected;
static uint32_t rate, sequence, request, started, tx_started, ready_at, next_due, period;
static uint32_t missed, failures;
static uint8_t pulses, captured_pulses;
_Alignas(uint32_t) static uint16_t samples[DRIVE_SAMPLE_COUNT];
static uint8_t tx[DRIVE_FRAME_SIZE], rx[DRIVE_COMMAND_SIZE];
static uint8_t replies[REPLY_COUNT][REPLY_SIZE];
static size_t rx_used, reply_head, reply_used;
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v) {
    for (uint32_t i=0; i<4U; ++i) { p[i]=(uint8_t)(v>>(8U*i)); }
}
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
uint32_t drive_crc32(const uint8_t *data, size_t length) {
    uint32_t crc=UINT32_MAX;
    for (size_t i=0; i<length; ++i) {
        crc^=data[i];
        for (uint32_t b=0; b<8U; ++b) { crc=(crc>>1)^((crc&1U)!=0U ? UINT32_C(0xedb88320):0U); }
    }
    return ~crc;
}
static uint8_t parity(uint8_t a, uint8_t b) {
    uint8_t v=(uint8_t)(a^b);
    v^=(uint8_t)(v>>4); v^=(uint8_t)(v>>2); v^=(uint8_t)(v>>1);
    return (uint8_t)(v&1U);
}
static drive_result reg_transfer(uint8_t address, bool read, uint8_t value, uint8_t *out) {
    uint8_t command[2]={(uint8_t)((address<<1)|(read?0x80U:0U)),value};
    uint8_t response[2]={0U,0U};
    command[0]|=(uint8_t)(parity(command[0],command[1])^1U);
    drive_result r=io->spi(command,response,SPI_TIMEOUT_MS);
    if (r!=DRIVE_OK) { return r; }
    if (parity(response[0],response[1])!=1U || (response[0]&0x80U)!=0U) { return DRIVE_PARITY; }
    *out=response[1];
    return DRIVE_OK;
}
static drive_result read_reg(uint8_t address, uint8_t *out) { return reg_transfer(address,true,0U,out); }
static drive_result write_reg(uint8_t address, uint8_t value, uint8_t mask) {
    uint8_t actual=0U;
    drive_result r=reg_transfer(address,false,value,&actual);
    if (r==DRIVE_OK) { r=read_reg(address,&actual); }
    if (r==DRIVE_OK && (actual&mask)!=(value&mask)) { r=DRIVE_MISMATCH; }
    return r;
}
static drive_result configure(void) {
    /* Provisional original-board analog profile; external VDRV disables internal regulator. */
    static const uint8_t profile[][3]={
        {REG_TOF,0x40,0xc3}, /* Standby clears stale pulse diagnostic latches. */
        {REG_BPF1,0x5f,0xff}, {REG_BPF2,0x00,0x3f},
        {REG_CTRL1,0xb7,0xff}, {REG_CTRL2,0x42,0xc7},
        {REG_CTRL3,0x01,0x1f}, {REG_VDRV,0x3f,0x7f},
        {REG_ECHO,0x00,0x1f}, {REG_ZC,0x00,0xff}, {REG_TOF,0x00,0xc3}
    };
    configured=false;
    drive_result r=io->stop();
    if (r!=DRIVE_OK) { return r; }
    io->delay_ms(100U);
    uint8_t id=0U, revision=0U;
    r=read_reg(REG_ID,&id);
    if (r==DRIVE_OK) { r=read_reg(REG_REV,&revision); }
    if (r!=DRIVE_OK) { return r; }
    if (id!=0xb9U || revision!=0x02U) { return DRIVE_DEVICE; }
    for (size_t i=0; i<sizeof(profile)/sizeof(profile[0]); ++i) {
        r=write_reg(profile[i][0],profile[i][1],profile[i][2]);
        if (r!=DRIVE_OK) { return r; }
    }
    r=write_reg(REG_BURST,pulses,0xffU);
    if (r==DRIVE_OK) { r=read_reg(REG_STATUS,&id); }
    if (r==DRIVE_OK && (id&1U)!=0U) { r=DRIVE_DEVICE; }
    if (r==DRIVE_OK) { configured=true; }
    return r;
}
static void fault(drive_result r) {
    drive_result stopped=io->stop();
    error=(stopped==DRIVE_OK)?r:stopped;
    state=DRIVE_FAULT; streaming=false; ++failures;
}
static void header(uint8_t *dst, uint8_t type, drive_result result, uint32_t id, uint16_t count) {
    memset(dst,0,DRIVE_HEADER_SIZE);
    memcpy(dst,"EFW1",4U); dst[4]=1U; dst[5]=type;
    put16(dst+6,(uint16_t)result); put32(dst+8,DRIVE_HEADER_SIZE+2U*(uint32_t)count+4U);
    put32(dst+12,id); put32(dst+16,sequence); put32(dst+20,rate);
    put16(dst+24,count); dst[26]=count!=0U?captured_pulses:pulses;
    dst[27]=(uint8_t)(1U|(streaming?2U:0U)|((uint32_t)state<<2)|((count!=0U && captured_pulses==0U)?16U:0U));
    put32(dst+28,CARRIER_HZ);
    /* Nominal timer phase relative to first ADC trigger, not calibrated aperture time. */
    put32(dst+32,(count!=0U && captured_pulses==0U)?0U:((rate==RATE_FAST)?10083U:9667U));
    put32(dst+36,missed); put32(dst+40,io->reset_cause); put32(dst+44,failures);
}
static void reply(uint8_t type, drive_result r, uint32_t id) {
    if (reply_used==REPLY_COUNT) { ++failures; return; }
    uint8_t *dst=replies[(reply_head+reply_used)%REPLY_COUNT];
    header(dst,type,r,id,0U); put32(dst+DRIVE_HEADER_SIZE,drive_crc32(dst,DRIVE_HEADER_SIZE));
    ++reply_used;
}
static drive_result launch(uint32_t id, bool baseline) {
    if (state!=DRIVE_IDLE || pending || reply_used!=0U) { return DRIVE_BUSY; }
    if (!configured) { return DRIVE_DEVICE; }
    if (!io->connected()) { return DRIVE_DISCONNECTED; }
    drive_result r=write_reg(REG_BURST,pulses,0xffU);
    if (r!=DRIVE_OK) { fault(r); return r; }
    if (!io->connected()) { return DRIVE_DISCONNECTED; }
    captured_pulses=baseline?0U:pulses;
    ++sequence; request=id; started=io->millis(); state=DRIVE_ACQUIRING;
    r=io->capture(samples,DRIVE_SAMPLE_COUNT,rate,captured_pulses);
    if (r!=DRIVE_OK) { fault(r); }
    return r;
}
static void command(void) {
    uint32_t id=get32(rx+8), arg=get32(rx+12);
    uint8_t op=rx[4];
    drive_result r=DRIVE_OK;
    /* STOP remains effective even if a hostile host fills the reply queue. */
    if (reply_used==REPLY_COUNT && op!=CMD_STOP) { ++failures; return; }
    if (rx[5]!=0U || rx[6]!=0U || rx[7]!=0U) { r=DRIVE_ARGUMENT; }
    else {
        switch (op) {
        case CMD_INFO: case CMD_STATUS: r=error; break;
        case CMD_CONFIG:
            if (state!=DRIVE_IDLE || streaming || pending) { r=DRIVE_BUSY; }
            else if ((arg&0xffU)==0U || (arg&0xffU)>20U || (arg>>8)>1U) { r=DRIVE_ARGUMENT; }
            else { pulses=(uint8_t)(arg&0xffU); rate=(arg>>8)!=0U?RATE_FAST:RATE_DIAGNOSTIC; }
            break;
        case CMD_CAPTURE: case CMD_BASELINE:
            r=streaming?DRIVE_BUSY:launch(id,op==CMD_BASELINE); break;
        case CMD_STREAM:
            if (arg<20U || arg>10000U) { r=DRIVE_ARGUMENT; }
            else if (state!=DRIVE_IDLE || streaming || pending) { r=DRIVE_BUSY; }
            else { streaming=true; period=arg; next_due=io->millis()+period; request=id; }
            break;
        case CMD_STOP:
            streaming=false; r=io->stop();
            if (r!=DRIVE_OK) { fault(r); }
            else if (state!=DRIVE_FAULT) { state=DRIVE_IDLE; error=DRIVE_OK; }
            break;
        case CMD_RECOVER:
            if (state==DRIVE_ACQUIRING || pending || streaming) { r=DRIVE_BUSY; }
            else { r=configure(); error=r; state=r==DRIVE_OK?DRIVE_IDLE:DRIVE_FAULT; }
            break;
        default: r=DRIVE_ARGUMENT; break;
        }
    }
    reply((op==CMD_INFO || op==CMD_STATUS)?FRAME_STATUS:FRAME_ACK,r,id);
}
drive_result drive_init(const drive_port *port) {
    if (port==NULL || port->spi==NULL || port->delay_ms==NULL || port->millis==NULL ||
        port->capture==NULL || port->capture_state==NULL || port->stop==NULL ||
        port->connected==NULL || port->usb_busy==NULL || port->usb_send==NULL) { return DRIVE_ARGUMENT; }
    io=port; state=DRIVE_FAULT; error=DRIVE_OK; streaming=false; pending=false; configured=false;
    was_connected=false; rate=RATE_FAST; pulses=5U; sequence=0U; request=0U;
    missed=0U; failures=0U; rx_used=0U; reply_head=0U; reply_used=0U;
    error=configure(); state=error==DRIVE_OK?DRIVE_IDLE:DRIVE_FAULT;
    return error;
}
void drive_receive(const uint8_t *data, size_t length) {
    if (io==NULL || data==NULL) { return; }
    for (size_t i=0; i<length; ++i) {
        rx[rx_used++]=data[i];
        if (rx_used==DRIVE_COMMAND_SIZE) {
            if (memcmp(rx,"EFC1",4U)==0 && drive_crc32(rx,16U)==get32(rx+16)) { command(); rx_used=0U; }
            else { memmove(rx,rx+1,DRIVE_COMMAND_SIZE-1U); rx_used=DRIVE_COMMAND_SIZE-1U; }
        }
    }
}
void drive_process(void) {
    if (io==NULL) { return; }
    uint32_t now=io->millis();
    if (!io->connected()) {
        if (was_connected || streaming || pending || state==DRIVE_ACQUIRING || state==DRIVE_READY) {
            drive_result r=io->stop();
            if (r!=DRIVE_OK) { fault(r); }
            else if (state!=DRIVE_FAULT) { state=DRIVE_IDLE; }
        }
        streaming=false; pending=false; reply_used=0U; rx_used=0U; was_connected=false; return;
    }
    was_connected=true;
    if (pending && !io->usb_busy()) { pending=false; }
    if (pending && now-tx_started>=TRANSPORT_TIMEOUT_MS) {
        if (state!=DRIVE_FAULT) { fault(DRIVE_TIMEOUT); }
        return; /* Retain the USB buffer until completion or disconnect. */
    }
    if (state==DRIVE_ACQUIRING) {
        capture_status s=io->capture_state();
        if (s==CAPTURE_DONE) {
            drive_result r=io->stop(); uint8_t flags=0U;
            if (r==DRIVE_OK) { r=read_reg(REG_STATUS,&flags); }
            if (r==DRIVE_OK && (flags&7U)!=0U) { r=DRIVE_DEVICE; }
            if (r==DRIVE_OK) { state=DRIVE_READY; ready_at=now; }
            else { fault(r); reply(FRAME_ERROR,error,request); }
        } else if (s!=CAPTURE_ACTIVE || now-started>=CAPTURE_TIMEOUT_MS) {
            fault(s==CAPTURE_ACTIVE?DRIVE_TIMEOUT:DRIVE_ADC); reply(FRAME_ERROR,error,request);
        }
    }
    if (state==DRIVE_READY && now-ready_at>=TRANSPORT_TIMEOUT_MS) {
        fault(DRIVE_TIMEOUT); reply(FRAME_ERROR,error,request);
    }
    if (!pending && (reply_used!=0U || state==DRIVE_READY)) {
        bool control=reply_used!=0U;
        uint16_t length=REPLY_SIZE;
        if (control) { memcpy(tx,replies[reply_head],REPLY_SIZE); }
        else {
            header(tx,FRAME_WAVE,DRIVE_OK,request,DRIVE_SAMPLE_COUNT);
            for (size_t i=0; i<DRIVE_SAMPLE_COUNT; ++i) {
                if (samples[i]>4095U) { fault(DRIVE_ADC); reply(FRAME_ERROR,error,request); return; }
                put16(tx+DRIVE_HEADER_SIZE+2U*i,samples[i]);
            }
            length=DRIVE_FRAME_SIZE; put32(tx+length-4U,drive_crc32(tx,(size_t)length-4U));
        }
        drive_result r=io->usb_send(tx,length);
        if (r==DRIVE_OK) {
            pending=true; tx_started=now;
            if (control) { reply_head=(reply_head+1U)%REPLY_COUNT; --reply_used; }
            else { state=DRIVE_IDLE; }
        } else if (r!=DRIVE_BUSY) { fault(r); }
    }
    if (streaming && now-next_due<UINT32_C(0x80000000)) {
        uint32_t slots=(now-next_due)/period+1U;
        next_due+=slots*period; missed+=slots-1U;
        drive_result r=launch(request,false);
        if (r==DRIVE_BUSY) { ++missed; }
        else if (r!=DRIVE_OK) { reply(FRAME_ERROR,r,request); }
    }
}
drive_status drive_get_state(void) { return state; }
drive_result drive_get_error(void) { return error; }
