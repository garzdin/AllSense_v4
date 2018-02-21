/*
 * serial.h
 *
 * Created: 2/21/2018 11:05:59 AM
 *  Author: Teodor
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

typedef enum SERIAL_RET_enum {
	ERROR,
	OK,
} SERIAL_RET_t;

typedef struct SERIAL_struct {
	USART_t * _usart;
	PORT_t * _port;
	uint16_t _baud;
	uint8_t _rxc_int_lvl;
	uint8_t _txc_int_lvl;
	uint8_t _rx_en;
	uint8_t _tx_en;
	uint16_t _rx_buf_size;
	uint16_t _tx_buf_size;
	uint8_t * _rx_buf;
	uint8_t * _tx_buf;
	uint8_t _rx_buf_ovf;
	#if RX_BUFFER_SIZE <= 256
	volatile uint8_t _rx_wr_index;
	volatile uint8_t _rx_rd_index;
	volatile uint8_t _tx_wr_index;
	volatile uint8_t _tx_rd_index;
	#else
	volatile uint16_t _rx_wr_index;
	volatile uint16_t _rx_rd_index;
	volatile uint16_t _tx_wr_index;
	volatile uint16_t _tx_rd_index;
	#endif
	#if RX_BUFFER_SIZE < 256
	volatile uint8_t _rx_counter;
	volatile uint8_t _tx_counter;
	#else
	volatile uint16_t _rx_counter;
	volatile uint16_t _tx_counter;
	#endif
} SERIAL_t;

void serial_rx_isr_handler (SERIAL_t * serial);
void serial_tx_isr_handler(SERIAL_t * serial);
SERIAL_RET_t serial_set_baud (SERIAL_t * serial, uint32_t * f_cpu, uint8_t * bscale, uint32_t * baud);
SERIAL_RET_t serial_set_tx (SERIAL_t * serial, uint8_t tx_state);
SERIAL_RET_t serial_set_rx (SERIAL_t * serial, uint8_t rx_state);
SERIAL_RET_t serial_listen (SERIAL_t * serial);
SERIAL_RET_t serial_available (SERIAL_t * serial);
SERIAL_RET_t serial_putchar (SERIAL_t * serial, uint8_t c);
uint8_t serial_getchar (SERIAL_t * serial, SERIAL_t * debug);
SERIAL_RET_t serial_puts (SERIAL_t * serial, char * str);
SERIAL_RET_t serial_gets (SERIAL_t * serial, uint8_t * buf, SERIAL_t * debug);
SERIAL_RET_t serial_init (SERIAL_t * serial, USART_t * usart, PORT_t * port, uint32_t baud, uint32_t f_cpu, uint8_t bscale);

#endif /* SERIAL_H_ */