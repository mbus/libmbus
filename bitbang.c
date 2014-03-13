#include "bitbang.h"

#include <string.h>

struct MBus_t* mbus;

static volatile enum MBus_state_t {
	IDLE,
	PREARB,
	ARBITRATION,
	PRIO_DRIVE,
	PRIO_LATCH,
	DRIVE_SHORT_ADDR,
	LATCH_SHORT_ADDR,
	DRIVE_LONG_ADDR,
	LATCH_LONG_ADDR,
	DRIVE_DATA,
	LATCH_DATA,
	REQUEST_INTERRUPT,
	REQUESTING_INTERRUPT,
	REQUESTED_INTERRUPT,
	PRE_BEGIN_CONTROL,
	BEGIN_CONTROL,
	DRIVE_CB0,
	LATCH_CB0,
	DRIVE_CB1,
	LATCH_CB1,
	DRIVE_IDLE,
	BEGIN_IDLE,
	ERROR
} state = IDLE;

static volatile enum MBus_logical_t {
	FORWARD,
	TRANSMIT,
	RECEIVE,
	RECEIVE_BROADCAST,
	INTERRUPTER,
} logical = FORWARD;

static volatile uint8_t last_clkin = 1;
static volatile uint8_t last_din = 1;
static volatile uint8_t last_dout = 1;
static volatile uint8_t interrupt_count = 0;
static volatile enum MBus_error_t error = MBUS_NO_ERROR;

static          uint8_t *tx_buf = NULL;
static          int      tx_length = 0;
static          uint8_t  tx_priority = 0;
static volatile uint8_t  tx_bit_idx = 0;
static volatile int      tx_byte_idx = 0;

static volatile uint32_t rx_addr = 0;
static volatile uint8_t  rx_bit_idx = 0;
static volatile int      rx_byte_idx = 0;
static          int      rx_buf_zero = 0;
static volatile int*     rx_buf_len = &rx_buf_zero;
static volatile uint8_t* rx_buf = NULL;

static volatile uint8_t  ack = 0;


static inline void SET_CLKOUT_TO(uint8_t val) {
	mbus->set_gpio_val(mbus->CLKOUT_gpio, val);
}
static inline void SET_CLKOUT_HIGH(void) {
	SET_CLKOUT_TO(1);
}
static inline void SET_CLKOUT_LOW(void) {
	SET_CLKOUT_TO(0);
}

static inline void SET_DOUT_TO(uint8_t val) {
	mbus->set_gpio_val(mbus->DOUT_gpio, val);
}
static inline void SET_DOUT_HIGH(void) {
	SET_DOUT_TO(1);
}
static inline void SET_DOUT_LOW(void) {
	SET_DOUT_TO(0);
}


void MBus_init(struct MBus_t *m) {
	mbus = m;

	state = IDLE;
	logical = FORWARD;
	last_clkin = 1;
	last_din = 1;
	last_dout = 1;
	interrupt_count = 0;
	error = MBUS_NO_ERROR;

	tx_buf = NULL;
	tx_length = 0;
	tx_priority = 0;
	tx_bit_idx = 0;
	tx_byte_idx = 0;

	rx_addr = 0;
	rx_bit_idx = 0;
	rx_byte_idx = 0;
	rx_buf_len = &rx_buf_zero;
	rx_buf = NULL;

	ack = 0;
}

void MBus_send(uint8_t* buf, int length, uint8_t is_priority) {
	tx_buf = buf;
	tx_length = length;
	tx_priority = is_priority;

	if (state == IDLE) {
		// It is safe to directly change logical model and drive DOUT
		// here. The state changes to PREARB at the falling edge of
		// clock the half-period before arbitration resolution
		logical = TRANSMIT;
		SET_DOUT_LOW();
	} else {
		// TODO: Handle TX request when bus is busy better. We could
		// probably check this status at the end of the current
		// transaction? Currently we just immediately fail.
		mbus->MBus_send_done(0);
	}
}

void MBus_CLKIN_int_handler(int CLKIN_val) {
	if (last_clkin == CLKIN_val) {
		if (state == ERROR) return;
		state = ERROR;
		error = MBUS_CLOCK_SYNCH_ERROR;
		return;
	}
	last_clkin = CLKIN_val;

	interrupt_count = 0;

	switch (state) {
		case IDLE:
			state = PREARB;
			tx_bit_idx = 0;
			tx_byte_idx = 0;
			rx_addr = 0;
			rx_bit_idx = 0;
			rx_byte_idx = 0;
			rx_buf_len = &rx_buf_zero;
			rx_buf = NULL;
			ack = 0;
			break;

		case PREARB:
			state = ARBITRATION;
			break;

		case ARBITRATION:
			state = PRIO_DRIVE;
			if (!last_din) {
				// Lost arbitration or didn't participate
				logical = FORWARD;
			} else {
				if (!last_dout) {
					// Won arbitration
					logical = TRANSMIT;
				} else {
					// Didn't participate
					logical = FORWARD;
				}
			}
			break;

		case PRIO_DRIVE:
			state = PRIO_LATCH;
			if (tx_priority) {
				SET_DOUT_HIGH();
			}
			break;

		case PRIO_LATCH:
			state = DRIVE_SHORT_ADDR;
			if (logical == TRANSMIT) {
				if (tx_priority) {
					// NOP, won prio arbitration
				} else {
					if (last_din) {
						// Lost to prio arb
						logical = FORWARD;
					} else {
						// NOP, won arbitration
					}
				}
			} else {
				if (tx_priority) {
					if (last_din) {
						// NOP, lost prio arbitration
					} else {
						// Won prio arbitration
						logical = TRANSMIT;
					}
				} else {
					// NOP, did not participate
				}
			}

			if (logical == TRANSMIT) state = DRIVE_DATA;
			break;

		// ADDR states only used in FWD/RX mode
		case DRIVE_SHORT_ADDR:
			state = LATCH_SHORT_ADDR;
			break;

		case LATCH_SHORT_ADDR:
			state = DRIVE_SHORT_ADDR;

			rx_addr <<= 1;
			rx_addr |= last_din;

			rx_bit_idx++;
			if (rx_bit_idx == 4) {
				if (rx_addr == 0xf) {
					state = DRIVE_LONG_ADDR;
				} else if (rx_addr == mbus->short_prefix) {
					logical = RECEIVE;
				} else if (rx_addr == 0) {
					logical = RECEIVE_BROADCAST;
				} else {
					logical = FORWARD;
				}
			} else if (rx_bit_idx == 8) {
				state = DRIVE_DATA;
				if (logical == RECEIVE_BROADCAST) {
					char channel = rx_addr & 0xf;
					if (mbus->broadcast_channels &
							(1 << channel)) {
						logical = RECEIVE;
					} else {
						logical = FORWARD;
					}
				}
				if (logical == RECEIVE) {
					if (mbus->recv_buf_0_len >= 1) {
						rx_buf_len = &mbus->recv_buf_0_len;
						rx_buf = mbus->recv_buf_0;
						memcpy(rx_buf, &rx_addr, 1);
					} else if (mbus->recv_buf_1_len >= 1) {
						rx_buf_len = &mbus->recv_buf_1_len;
						rx_buf = mbus->recv_buf_1;
						memcpy(rx_buf, &rx_addr, 1);
					} else {
						// TODO: Assert interrupt
					}
					rx_byte_idx = 1;
					rx_bit_idx = 0;
				}
			}
			break;

		case DRIVE_LONG_ADDR:
			state = LATCH_LONG_ADDR;
			break;

		case LATCH_LONG_ADDR:
			state = DRIVE_LONG_ADDR;

			rx_addr <<= 1;
			rx_addr |= last_din;

			rx_bit_idx++;
			if (rx_bit_idx == 28) {
				if ((rx_addr & 0xffffff) == mbus->full_prefix) {
					logical = RECEIVE;
				} else if ((rx_addr & 0xffffff) == 0) {
					logical = RECEIVE_BROADCAST;
				} else {
					logical = FORWARD;
				}
			} else if (rx_bit_idx == 32) {
				state = DRIVE_DATA;
				if (logical == RECEIVE_BROADCAST) {
					char channel = rx_addr & 0xf;
					if (mbus->broadcast_channels &
							(1 << channel)) {
						logical = RECEIVE;
					} else {
						logical = FORWARD;
					}
				}
				if (logical == RECEIVE) {
					if (mbus->recv_buf_0_len >= 4) {
						rx_buf_len = &mbus->recv_buf_0_len;
						rx_buf = mbus->recv_buf_0;
						memcpy(rx_buf, &rx_addr, 4);
					} else if (mbus->recv_buf_1_len >= 4) {
						rx_buf_len = &mbus->recv_buf_1_len;
						rx_buf = mbus->recv_buf_1;
						memcpy(rx_buf, &rx_addr, 4);
					}
					rx_byte_idx = 4;
					rx_bit_idx = 0;
				}
			}
			break;

		case DRIVE_DATA:
			state = LATCH_DATA;
			if (logical == TRANSMIT) {
				uint8_t bit;
				bit = !!(tx_buf[tx_byte_idx] & (1 << tx_bit_idx));
				SET_DOUT_TO(bit);
				tx_bit_idx++;
				if (tx_bit_idx == 8) {
					tx_bit_idx = 0;
					tx_byte_idx++;
				}
			}
			break;

		case LATCH_DATA:
			state = DRIVE_DATA;
			if (logical == RECEIVE) {
				if (rx_byte_idx > *rx_buf_len) {
					state = REQUEST_INTERRUPT;
					logical = TRANSMIT;
					error = MBUS_RECV_OVERFLOW;
					break;
				}
				rx_buf[rx_byte_idx] |= last_din << rx_bit_idx;
				rx_bit_idx++;
				if (rx_bit_idx == 8) {
					rx_bit_idx = 0;
					rx_byte_idx++;
				}
			}
			if (logical == TRANSMIT) {
				if (tx_byte_idx == tx_length) {
					state = REQUEST_INTERRUPT;
					error = MBUS_NO_ERROR;
				}
			}
			break;

		case REQUEST_INTERRUPT:
			if (last_clkin == 0) state = REQUESTING_INTERRUPT;
			break;

		case REQUESTING_INTERRUPT:
			if (last_clkin == 0) state = REQUESTED_INTERRUPT;
			break;

		case REQUESTED_INTERRUPT:
			break;

		case PRE_BEGIN_CONTROL:
			state = BEGIN_CONTROL;

		case BEGIN_CONTROL:
			state = DRIVE_CB0;
			break;

		case DRIVE_CB0:
			state = LATCH_CB0;
			if (logical == INTERRUPTER) {
				if (error == MBUS_NO_ERROR) {
					SET_DOUT_HIGH(); // EoM;
				} else {
					SET_DOUT_LOW(); // !EoM;
				}
			}
			break;

		case LATCH_CB0:
			state = DRIVE_CB1;
			ack = last_din;
			if (logical == RECEIVE) {
				// Swtich to TX mode to send CB1
				logical = TRANSMIT;
			} else if (error == MBUS_NO_ERROR) {
				logical = FORWARD;
			}
			break;

		case DRIVE_CB1:
			state = LATCH_CB1;
			if (logical == INTERRUPTER) {
				if (error == MBUS_RECV_OVERFLOW) {
					SET_DOUT_HIGH(); // Tx/Rx Error
				}
			} else if (logical == TRANSMIT) {
				// Actually the receiver here, but TX'ing CB1
				if (ack == 1) {
					SET_DOUT_LOW(); // Ack
				}
			}
			break;

		case LATCH_CB1:
			state = DRIVE_IDLE;
			logical = FORWARD;
			if (tx_byte_idx > 0) {
				// We transmitted
				ack = last_din;
			}
			break;

		case DRIVE_IDLE:
			state = BEGIN_IDLE;
			break;

		case BEGIN_IDLE:
			if (last_din == 1) {
				state = IDLE;
			} else {
				state = PREARB;
			}
			break;

		case ERROR:
			break;
	}

	if (
			(state == REQUEST_INTERRUPT) ||
			(state == REQUESTING_INTERRUPT) ||
			(state == REQUESTED_INTERRUPT)
	   ) {
		SET_CLKOUT_HIGH();
	} else {
		SET_CLKOUT_TO(last_clkin);
	}

	if (state == BEGIN_IDLE) {
		if (error != MBUS_NO_ERROR) {
			mbus->MBus_error(error);
		} else if (tx_byte_idx > 0) {
			mbus->MBus_send_done(tx_byte_idx);
		} else if (rx_byte_idx > 0) {
			*rx_buf_len = -rx_byte_idx;
			mbus->MBus_recv(rx_buf_len == &mbus->recv_buf_1_len);
		}
	}
}

void MBus_DIN_int_handler(int DIN_val) {
	if (last_din == DIN_val) {
		if (state == ERROR) return;
		state = ERROR;
		error = MBUS_DATA_SYNCH_ERROR;
		return;
	}
	last_din = DIN_val;

	if (last_din) interrupt_count++;

	if (interrupt_count >= 3) {
		if (state == REQUESTED_INTERRUPT) {
			logical = INTERRUPTER;
		}
		state = PRE_BEGIN_CONTROL;
	}

	if (state < REQUEST_INTERRUPT) {
		if (logical != TRANSMIT) {
			SET_DOUT_TO(last_din);
		}
	} else if (state <= BEGIN_CONTROL) {
		SET_DOUT_TO(last_din);
	} else {
		if (logical != TRANSMIT) {
			SET_DOUT_TO(last_din);
		}
	}
}
