#ifndef BOARD_H
#define BOARD_H
#include "drive.h"
/** Provisional F446RC / 25 MHz board; initialize with outputs inhibited. */
drive_result board_init(void);
/** Drain USB commands into drive_receive in foreground; never blocks for USB. */
void board_poll(void);
/** HAL-backed port, valid after successful board_init. */
extern drive_port board_port;
#endif
