#ifndef BITBANG_H
#define BITBANG_H

#include <stdint.h>

/* This file is written to be architecture and platform independent. To
 * facilitate this, we define a simple, standard interface to the required
 * hardware components. Platforms will need to provide a small shim layer.
 *
 * Platforms MUST provide wrappers for the interrupt handlers to support
 * platform-specific interrupt entry / exit procedures.
 *
 * The interrupt handlers are NOT designed to be re-entrant. Shims should not
 * clear the interrupt until the MBus handler returns (handlers are very
 * lightweight).
 *
 * The MBus interrupt handlers DO NOT clear the interrupt (if appropriate).
 *
 * Usage:
 *   Upon startup, platforms should call MBus_init. MBus_init will call GPIO
 *   direction and interrupt functions as appropriate.
 *
 *   MBus is event-based and requires running the MBus_run() loop to handle
 *   events. MBus_run shall return true if there are more events to process
 *   and false otherwise, that is:
 *     while(MBus_run()) ;
 *   is appropriate for exhausting bus events. Once MBus_run returns false,
 *   subsequent calls to MBus_run MUST return false until one of MBus_send,
 *   MBus_DIN_int_handler, or MBus_CLKIN_int_hander is called.
 *
 *   MBus_send will arbitrate for the bus and then write an array of bytes
 *   directly onto the wires (that is, the address must be included as the
 *   first byte(s) given to MBus_send). Upon completion of transmission the
 *   MBus_send_done callback will be called with the result. MBus_send_done
 *   should be treated as an interrupt and perform minimal processing,
 *   although it is called from the MBus_run context, not an interrupt
 *   handler. Only one call to MBus_send may be "live" at any time. The effect
 *   of multiple calls to MBus_send without an intervening MBus_send_done is
 *   undefined.
 *
 *   Upon receipt of a whole message, the next call to MBus_run *after* the
 *   complete MBus control sequence will call the MBus_recv callback function.
 *   This function should be treated as an interrupt and perform minimal
 *   processing, although it is not called from an interrupt context.
 */

enum MBus_error_t {
	MBUS_NO_ERROR,
	MBUS_CLOCK_SYNCH_ERROR,
	MBUS_DATA_SYNCH_ERROR,
	MBUS_RECV_OVERFLOW,
	MBUS_INTERRUPTED,
};

struct MBus_t {
	int CLKIN_gpio;      // GPIO pin index assigned to CLKIN
	int CLKOUT_gpio;     // GPIO pin index assigned to CLKOUT
	int DIN_gpio;        // GPIO pin index assigned to DIN
	int DOUT_gpio;       // GPIO pin index assigned to DOUT

	// Boolean. Set false if only listening on the bus
	uint8_t participate_in_enumeration;

	// Bit Vector. Broadcast channels to subscribe to.
	uint16_t broadcast_channels;

	// Boolean. Call MBus_recv for all messages. Does not ACK messages
	// that would otherwise have been ignored.
	uint8_t promiscuous_mode;

	// [OPT] Static short prefix. This value will be overridden if
	// enumeration is performed to hold the current short prefix. Only the
	// bottom four bits of this value are signficant.
	uint8_t short_prefix;
	// Full prefix. Only the bottom 6 bytes of this value are significant.
	//   _Note:_ The most-significant byte is reserved and should be 0.
	uint32_t full_prefix;

	void (*set_gpio_dir)(int gpio_idx, int is_output);
	void (*set_gpio_int)(int gpio_idx, int is_interrupt_active);
	void (*set_gpio_val)(int gpio_idx, int is_high);

	void (*MBus_send_done)(int bytes_sent);

	volatile uint8_t* recv_buf_0; // Valid iff recv_buf_0_len > 0
	volatile int recv_buf_0_len;
	volatile uint8_t* recv_buf_1; // Valid iff recv_buf_1_len > 0
	volatile int recv_buf_1_len;
	void (*MBus_recv)(int recv_buf_idx); // Will be 0 or 1

	void (*MBus_error)(enum MBus_error_t);
};

void MBus_init(struct MBus_t *); // Pointer must remain valid forever
void MBus_run(void);
void MBus_send(uint8_t* buf, int length, uint8_t is_priority);
  // buf pointer must reamin valid until send_done is called

void MBus_DIN_int_handler(int DIN_val);
void MBus_CLKIN_int_handler(int CLKIN_val);

#endif // BITBANG_H
