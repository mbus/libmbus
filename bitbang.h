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
 * MBus does not configure GPIO modes in any way. The user must configure
 * interrupt pins and ensure that the pins assigned as DOUT and CLKOUT are set
 * up as output pins.
 *
 * MBus uses some static global variables. Do not try to instantiate more
 * than one instance of MBus, it won't work (this could obviously be worked
 * around if necessary, but it is not a priority).
 *
 * Usage:
 *   Upon startup, platforms should call MBus_init. MBus_init takes a reference
 *   to a configuration structure (struct MBus_t). This structure must remain
 *   valid forever. It is safe to modify reasonable things (e.g. the send done
 *   callback when no send is in progress) during runtime.
 *
 *   MBus_send will arbitrate for the bus and then write an array of bytes
 *   directly onto the wires (that is, the address must be included as the
 *   first byte(s) given to MBus_send). Upon completion of transmission the
 *   MBus_send_done callback will be called with the result. MBus_send_done
 *   should be treated as an interrupt and perform minimal processing.
 *   Only one call to MBus_send may be "live" at any time. The effect of
 *   multiple calls to MBus_send without waiting for an intervening
 *   MBus_send_done is undefined.
 *
 *   The MBus struct contains two buffers for receiving incoming messages. A
 *   buffer is considered valid for use if its length field is greater than
 *   zero. A valid buffer may never be invalidated by the client library.
 *   When MBus uses a buffer to receive a message, it updates the length field
 *   to be -1 * the number of bytes received (e.g. a 8 byte message would have a
 *   length of -8). Note that this buffer is now marked as invalid and the
 *   client may do anything with the buffer. To mark a buffer as valid again,
 *   the client simply sets the length to a positive value.
 *   If no buffers are available when a message is addressed to this library,
 *   MBus will Interrupt and NAK the message sender.
 *   Upon receipt of a whole message, MBus_recv callback is called. This
 *   function should be treated as an interrupt and perform minimal processing.
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
