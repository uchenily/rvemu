#ifndef _UART_H_
#define _UART_H_

#include "platform.h"

// see http://byterunner.com/16550.html
#define UART0_BASE (0x10000000ul)
#define UART0_RHR SFR8(UART0_BASE + 0) // Receive Holding Register (read mode)
#define UART0_THR SFR8(UART0_BASE + 0) // Transmit Holding Register (write mode)
#define UART0_DLL SFR8(UART0_BASE + 0) // LSB of Divisor Latch (write mode)
#define UART0_IER SFR8(UART0_BASE + 1) // Interrupt Enable Register (write mode)
#define UART0_DLM SFR8(UART0_BASE + 1) // MSB of Divisor Latch (write mode)
#define UART0_FCR SFR8(UART0_BASE + 2) // FIFO Control Register (write mode)
#define UART0_ISR SFR8(UART0_BASE + 2) // Interrupt Status Register (read mode)
#define UART0_LCR SFR8(UART0_BASE + 3) // Line Control Register
#define UART0_MCR SFR8(UART0_BASE + 4) // Modem Control Register
#define UART0_LSR SFR8(UART0_BASE + 5) // Line Status Register
#define UART0_MSR SFR8(UART0_BASE + 6) // Modem Status Register
#define UART0_SPR SFR8(UART0_BASE + 7) // ScratchPad Register

#define LCR_BAUD_LATCH (1 << 7) // special mode to set baud rate
#define LCR_8BITS                                                              \
    (3 << 0) // set word length to 8bits, no parity, and then leave set-baud
             // mode

/*
 * POWER UP DEFAULTS
 * IER = 0: TX/RX holding register interrupts are both disabled
 * ISR = 1: no interrupt pending
 * LCR = 0
 * MCR = 0
 * LSR = 60 HEX
 * MSR = BITS 0-3 = 0, BITS 4-7 = inputs
 * FCR = 0
 * TX = High
 * OP1 = High
 * OP2 = High
 * RTS = High
 * DTR = High
 * RXRDY = High
 * TXRDY = Low
 * INT = Low
 */

/*
 * LINE STATUS REGISTER (LSR)
 * LSR BIT 0:
 * 0 = no data in receive holding register or FIFO.
 * 1 = data has been receive and saved in the receive holding register or FIFO.
 * ......
 * LSR BIT 5:
 * 0 = transmit holding register is full. 16550 will not accept any data for
 * transmission. 1 = transmitter hold register (or FIFO) is empty. CPU can load
 * the next character.
 * ......
 */
#define LSR_RX_READY (1 << 0)
#define LSR_TX_IDLE (1 << 5)

void uart_init(void);
void uart_putc(char c);
char uart_getc(void);
void uart_puts(const char *s);

#endif /* _UART_H_ */
