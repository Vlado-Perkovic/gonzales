#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Shell/console (UART0). Console task owns command execution; long-running
 * operations (run/mon) poll console_poll_stop() so that a "stop" line typed
 * mid-run aborts them. Input other than "stop" during a run is discarded. */
void console_start(void);

/* Drain pending RX bytes; update + return the sticky stop flag. */
bool console_poll_stop(void);
/* Clear the sticky stop flag and scan buffer (call before each command). */
void console_clear_stop(void);
