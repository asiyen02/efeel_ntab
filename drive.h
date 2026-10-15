#ifndef DRIVE_H
#define DRIVE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define DRIVE_SAMPLE_COUNT 2400U
#define DRIVE_HEADER_SIZE 48U
#define DRIVE_FRAME_SIZE (DRIVE_HEADER_SIZE + 2U * DRIVE_SAMPLE_COUNT + 4U)
#define DRIVE_COMMAND_SIZE 20U
/** All APIs and port functions execute in the foreground, never in an ISR. */
typedef enum { DRIVE_OK, DRIVE_BUSY, DRIVE_ARGUMENT, DRIVE_TIMEOUT, DRIVE_IO,
    DRIVE_PARITY, DRIVE_DEVICE, DRIVE_MISMATCH, DRIVE_ADC, DRIVE_DISCONNECTED,
    DRIVE_ABORTED, DRIVE_OVERFLOW } drive_result;
typedef enum { DRIVE_FAULT, DRIVE_IDLE, DRIVE_ACQUIRING, DRIVE_READY } drive_status;
typedef enum { CAPTURE_IDLE, CAPTURE_ACTIVE, CAPTURE_DONE, CAPTURE_ERROR } capture_status;
typedef struct {
    drive_result (*spi)(const uint8_t tx[2], uint8_t rx[2], uint32_t timeout_ms);
    void (*delay_ms)(uint32_t ms);
    uint32_t (*millis)(void);
    drive_result (*capture)(uint16_t *samples, uint16_t count, uint32_t rate, uint8_t pulses);
    capture_status (*capture_state)(void);
    drive_result (*stop)(void); /* Drain DMA/IRQs and return IO1/IO2 high. */
    bool (*connected)(void); /* False means USB no longer owns any TX storage. */
    bool (*usb_busy)(void);
    drive_result (*usb_send)(const uint8_t *data, uint16_t length);
    /* On OK, USB retains data until !usb_busy() or !connected(). */
    uint32_t reset_cause;
} drive_port;
/** Initialize once, disarmed. The port must remain valid for program lifetime. */
drive_result drive_init(const drive_port *port);
/** Feed arbitrary command fragments copied from a bounded ISR receive queue. */
void drive_receive(const uint8_t *data, size_t length);
/** Advance acquisition/transport; call frequently, including while USB is busy. */
void drive_process(void);
drive_status drive_get_state(void);
drive_result drive_get_error(void);
/** CRC-32/ISO-HDLC; no allocation, used by command and response framing. */
uint32_t drive_crc32(const uint8_t *data, size_t length);
#endif
