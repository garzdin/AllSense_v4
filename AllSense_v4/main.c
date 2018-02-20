// I/O Registers definitions
#include <avr/io.h>
#include <avr/interrupt.h>

// Declare your global variables here

// System Clocks initialization
void system_clocks_init(void)
{
	unsigned char n,s;

	// Optimize for speed
	s=SREG;
	// Disable interrupts
	asm("cli");

	// External 7372.800 kHz oscillator initialization
	// Crystal oscillator increased drive current: Off
	// External Clock Source - Startup Time: 0.4-16 MHz Quartz Crystal - 16k CLK
	OSC.XOSCCTRL=OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
	// Enable the external oscillator/clock source
	OSC.CTRL|=OSC_XOSCEN_bm;

	// System Clock prescaler A division factor: 1
	// System Clock prescalers B & C division factors: B:1, C:1
	// ClkPer4: 7372.800 kHz
	// ClkPer2: 7372.800 kHz
	// ClkPer:  7372.800 kHz
	// ClkCPU:  7372.800 kHz
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;

	// Wait for the external oscillator to stabilize
	while ((OSC.STATUS & OSC_XOSCRDY_bm)==0);

	// Select the system clock source: External Oscillator or Clock
	n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_XOSC_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=n;

	// Disable the unused oscillators: 2 MHz, 32 MHz, internal 32 kHz, PLL
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_PLLEN_bm);

	// ClkPer output disabled
	PORTCFG.CLKEVOUT&= ~(PORTCFG_CLKOUTSEL_gm | PORTCFG_CLKOUT_gm);
	// Restore interrupts enabled/disabled state
	SREG=s;
}

// Event System initialization
void event_system_init(void)
{
	// Event System Channel 0 source: None
	EVSYS.CH0MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 1 source: None
	EVSYS.CH1MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 2 source: None
	EVSYS.CH2MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 3 source: None
	EVSYS.CH3MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 4 source: None
	EVSYS.CH4MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 5 source: None
	EVSYS.CH5MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 6 source: None
	EVSYS.CH6MUX=EVSYS_CHMUX_OFF_gc;
	// Event System Channel 7 source: None
	EVSYS.CH7MUX=EVSYS_CHMUX_OFF_gc;

	// Event System Channel 0 Digital Filter Coefficient: 1 Sample
	// Quadrature Decoder: Off
	EVSYS.CH0CTRL=(EVSYS.CH0CTRL & (~(EVSYS_QDIRM_gm | EVSYS_QDIEN_bm | EVSYS_QDEN_bm | EVSYS_DIGFILT_gm))) |
	EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 1 Digital Filter Coefficient: 1 Sample
	EVSYS.CH1CTRL=EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 2 Digital Filter Coefficient: 1 Sample
	// Quadrature Decoder: Off
	EVSYS.CH2CTRL=(EVSYS.CH2CTRL & (~(EVSYS_QDIRM_gm | EVSYS_QDIEN_bm | EVSYS_QDEN_bm | EVSYS_DIGFILT_gm))) |
	EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 3 Digital Filter Coefficient: 1 Sample
	EVSYS.CH3CTRL=EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 4 Digital Filter Coefficient: 1 Sample
	// Quadrature Decoder: Off
	EVSYS.CH4CTRL=(EVSYS.CH4CTRL & (~(EVSYS_QDIRM_gm | EVSYS_QDIEN_bm | EVSYS_QDEN_bm | EVSYS_DIGFILT_gm))) |
	EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 5 Digital Filter Coefficient: 1 Sample
	EVSYS.CH5CTRL=EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 6 Digital Filter Coefficient: 1 Sample
	EVSYS.CH6CTRL=EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel 7 Digital Filter Coefficient: 1 Sample
	EVSYS.CH7CTRL=EVSYS_DIGFILT_1SAMPLE_gc;

	// Event System Channel output: Disabled
	PORTCFG.CLKEVOUT&= ~PORTCFG_EVOUT_gm;
	PORTCFG.EVOUTSEL&= ~PORTCFG_EVOUTSEL_gm;
}

// Ports initialization
void ports_init(void)
{
	// PORTA initialization
	// OUT register
	PORTA.OUT=0x00;
	// Pin0: Input
	// Pin1: Input
	// Pin2: Input
	// Pin3: Input
	// Pin4: Input
	// Pin5: Input
	// Pin6: Input
	// Pin7: Input
	PORTA.DIR=0x00;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTA.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTA.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin2 Output/Pull configuration: Totempole/No
	// Pin2 Input/Sense configuration: Sense both edges
	// Pin2 Inverted: Off
	// Pin2 Slew Rate Limitation: Off
	PORTA.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin3 Output/Pull configuration: Totempole/No
	// Pin3 Input/Sense configuration: Sense both edges
	// Pin3 Inverted: Off
	// Pin3 Slew Rate Limitation: Off
	PORTA.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin4 Output/Pull configuration: Totempole/No
	// Pin4 Input/Sense configuration: Sense both edges
	// Pin4 Inverted: Off
	// Pin4 Slew Rate Limitation: Off
	PORTA.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin5 Output/Pull configuration: Totempole/No
	// Pin5 Input/Sense configuration: Sense both edges
	// Pin5 Inverted: Off
	// Pin5 Slew Rate Limitation: Off
	PORTA.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin6 Output/Pull configuration: Totempole/No
	// Pin6 Input/Sense configuration: Sense both edges
	// Pin6 Inverted: Off
	// Pin6 Slew Rate Limitation: Off
	PORTA.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin7 Output/Pull configuration: Totempole/No
	// Pin7 Input/Sense configuration: Sense both edges
	// Pin7 Inverted: Off
	// Pin7 Slew Rate Limitation: Off
	PORTA.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTA.INTCTRL=(PORTA.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	// Pin2 Pin Change interrupt 0: Off
	// Pin3 Pin Change interrupt 0: Off
	// Pin4 Pin Change interrupt 0: Off
	// Pin5 Pin Change interrupt 0: Off
	// Pin6 Pin Change interrupt 0: Off
	// Pin7 Pin Change interrupt 0: Off
	PORTA.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	// Pin2 Pin Change interrupt 1: Off
	// Pin3 Pin Change interrupt 1: Off
	// Pin4 Pin Change interrupt 1: Off
	// Pin5 Pin Change interrupt 1: Off
	// Pin6 Pin Change interrupt 1: Off
	// Pin7 Pin Change interrupt 1: Off
	PORTA.INT1MASK=0x00;

	// PORTB initialization
	// OUT register
	PORTB.OUT=0x00;
	// Pin0: Input
	// Pin1: Input
	// Pin2: Input
	// Pin3: Input
	PORTB.DIR=0x04;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTB.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTB.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin2 Output/Pull configuration: Totempole/No
	// Pin2 Input/Sense configuration: Sense both edges
	// Pin2 Inverted: Off
	// Pin2 Slew Rate Limitation: Off
	PORTB.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin3 Output/Pull configuration: Totempole/No
	// Pin3 Input/Sense configuration: Sense both edges
	// Pin3 Inverted: Off
	// Pin3 Slew Rate Limitation: Off
	PORTB.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTB.INTCTRL=(PORTB.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	// Pin2 Pin Change interrupt 0: Off
	// Pin3 Pin Change interrupt 0: Off
	PORTB.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	// Pin2 Pin Change interrupt 1: Off
	// Pin3 Pin Change interrupt 1: Off
	PORTB.INT1MASK=0x00;

	// PORTC initialization
	// OUT register
	PORTC.OUT=0x08;
	// Pin0: Input
	// Pin1: Input
	// Pin2: Input
	// Pin3: Output
	// Pin4: Input
	// Pin5: Input
	// Pin6: Input
	// Pin7: Input
	PORTC.DIR=0x08;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTC.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTC.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin2 Output/Pull configuration: Totempole/No
	// Pin2 Input/Sense configuration: Sense both edges
	// Pin2 Inverted: Off
	// Pin2 Slew Rate Limitation: Off
	PORTC.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin3 Output/Pull configuration: Totempole/No
	// Pin3 Input/Sense configuration: Sense both edges
	// Pin3 Inverted: Off
	// Pin3 Slew Rate Limitation: Off
	PORTC.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin4 Output/Pull configuration: Totempole/No
	// Pin4 Input/Sense configuration: Sense both edges
	// Pin4 Inverted: Off
	// Pin4 Slew Rate Limitation: Off
	PORTC.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin5 Output/Pull configuration: Totempole/No
	// Pin5 Input/Sense configuration: Sense both edges
	// Pin5 Inverted: Off
	// Pin5 Slew Rate Limitation: Off
	PORTC.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin6 Output/Pull configuration: Totempole/No
	// Pin6 Input/Sense configuration: Sense both edges
	// Pin6 Inverted: Off
	// Pin6 Slew Rate Limitation: Off
	PORTC.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin7 Output/Pull configuration: Totempole/No
	// Pin7 Input/Sense configuration: Sense both edges
	// Pin7 Inverted: Off
	// Pin7 Slew Rate Limitation: Off
	PORTC.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// PORTC Peripheral Output Remapping
	// OC0A Output: Pin 0
	// OC0B Output: Pin 1
	// OC0C Output: Pin 2
	// OC0D Output: Pin 3
	// USART0 XCK: Pin 1
	// USART0 RXD: Pin 2
	// USART0 TXD: Pin 3
	// SPI MOSI: Pin 5
	// SPI SCK: Pin 7
	PORTC.REMAP=(0<<PORT_SPI_bp) | (0<<PORT_USART0_bp) | (0<<PORT_TC0D_bp) | (0<<PORT_TC0C_bp) | (0<<PORT_TC0B_bp) | (0<<PORT_TC0A_bp);
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTC.INTCTRL=(PORTC.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	// Pin2 Pin Change interrupt 0: Off
	// Pin3 Pin Change interrupt 0: Off
	// Pin4 Pin Change interrupt 0: Off
	// Pin5 Pin Change interrupt 0: Off
	// Pin6 Pin Change interrupt 0: Off
	// Pin7 Pin Change interrupt 0: Off
	PORTC.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	// Pin2 Pin Change interrupt 1: Off
	// Pin3 Pin Change interrupt 1: Off
	// Pin4 Pin Change interrupt 1: Off
	// Pin5 Pin Change interrupt 1: Off
	// Pin6 Pin Change interrupt 1: Off
	// Pin7 Pin Change interrupt 1: Off
	PORTC.INT1MASK=0x00;

	// PORTD initialization
	// OUT register
	PORTD.OUT=0x00;
	// Pin0: Output
	// Pin1: Output
	// Pin2: Input
	// Pin3: Input
	// Pin4: Input
	// Pin5: Input
	// Pin6: Input
	// Pin7: Input
	PORTD.DIR=0x03;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTD.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTD.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin2 Output/Pull configuration: Totempole/No
	// Pin2 Input/Sense configuration: Sense both edges
	// Pin2 Inverted: Off
	// Pin2 Slew Rate Limitation: Off
	PORTD.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin3 Output/Pull configuration: Totempole/No
	// Pin3 Input/Sense configuration: Sense both edges
	// Pin3 Inverted: Off
	// Pin3 Slew Rate Limitation: Off
	PORTD.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin4 Output/Pull configuration: Totempole/No
	// Pin4 Input/Sense configuration: Sense both edges
	// Pin4 Inverted: Off
	// Pin4 Slew Rate Limitation: Off
	PORTD.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin5 Output/Pull configuration: Totempole/No
	// Pin5 Input/Sense configuration: Sense both edges
	// Pin5 Inverted: Off
	// Pin5 Slew Rate Limitation: Off
	PORTD.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin6 Output/Pull configuration: Totempole/No
	// Pin6 Input/Sense configuration: Sense both edges
	// Pin6 Inverted: Off
	// Pin6 Slew Rate Limitation: Off
	PORTD.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin7 Output/Pull configuration: Totempole/No
	// Pin7 Input/Sense configuration: Sense both edges
	// Pin7 Inverted: Off
	// Pin7 Slew Rate Limitation: Off
	PORTD.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTD.INTCTRL=(PORTD.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	// Pin2 Pin Change interrupt 0: Off
	// Pin3 Pin Change interrupt 0: Off
	// Pin4 Pin Change interrupt 0: Off
	// Pin5 Pin Change interrupt 0: Off
	// Pin6 Pin Change interrupt 0: Off
	// Pin7 Pin Change interrupt 0: Off
	PORTD.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	// Pin2 Pin Change interrupt 1: Off
	// Pin3 Pin Change interrupt 1: Off
	// Pin4 Pin Change interrupt 1: Off
	// Pin5 Pin Change interrupt 1: Off
	// Pin6 Pin Change interrupt 1: Off
	// Pin7 Pin Change interrupt 1: Off
	PORTD.INT1MASK=0x00;

	// PORTE initialization
	// OUT register
	PORTE.OUT=0x08;
	// Pin0: Input
	// Pin1: Input
	// Pin2: Input
	// Pin3: Output
	PORTE.DIR=0x08;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTE.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTE.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin2 Output/Pull configuration: Totempole/No
	// Pin2 Input/Sense configuration: Sense both edges
	// Pin2 Inverted: Off
	// Pin2 Slew Rate Limitation: Off
	PORTE.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin3 Output/Pull configuration: Totempole/No
	// Pin3 Input/Sense configuration: Sense both edges
	// Pin3 Inverted: Off
	// Pin3 Slew Rate Limitation: Off
	PORTE.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTE.INTCTRL=(PORTE.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	// Pin2 Pin Change interrupt 0: Off
	// Pin3 Pin Change interrupt 0: Off
	PORTE.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	// Pin2 Pin Change interrupt 1: Off
	// Pin3 Pin Change interrupt 1: Off
	PORTE.INT1MASK=0x00;

	// PORTR initialization
	// OUT register
	PORTR.OUT=0x00;
	// Pin0: Input
	// Pin1: Input
	PORTR.DIR=0x00;
	// Pin0 Output/Pull configuration: Totempole/No
	// Pin0 Input/Sense configuration: Sense both edges
	// Pin0 Inverted: Off
	// Pin0 Slew Rate Limitation: Off
	PORTR.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Pin1 Output/Pull configuration: Totempole/No
	// Pin1 Input/Sense configuration: Sense both edges
	// Pin1 Inverted: Off
	// Pin1 Slew Rate Limitation: Off
	PORTR.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	// Interrupt 0 level: Disabled
	// Interrupt 1 level: Disabled
	PORTR.INTCTRL=(PORTR.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	// Pin0 Pin Change interrupt 0: Off
	// Pin1 Pin Change interrupt 0: Off
	PORTR.INT0MASK=0x00;
	// Pin0 Pin Change interrupt 1: Off
	// Pin1 Pin Change interrupt 1: Off
	PORTR.INT1MASK=0x00;
}

// Virtual Ports initialization
void vports_init(void)
{
	// PORTA mapped to VPORT0
	// PORTB mapped to VPORT1
	PORTCFG.VPCTRLA=PORTCFG_VP13MAP_PORTB_gc | PORTCFG_VP02MAP_PORTA_gc;
	// PORTC mapped to VPORT2
	// PORTD mapped to VPORT3
	PORTCFG.VPCTRLB=PORTCFG_VP13MAP_PORTD_gc | PORTCFG_VP02MAP_PORTC_gc;
}

// LED routine
void led_state(uint8_t state) {
	switch (state) {
		case 0:
			PORTB.OUT &= ~0x04;
		break;
		case 1:
			PORTB.OUT |= 0x04;
		break;
		default: break;
	}
}

// USARTC0 initialization
void usartc0_init(void)
{
	// Note: The correct PORTC direction for the RxD, TxD and XCK signals
	// is configured in the ports_init function.

	// Transmitter is enabled
	// Set TxD=1
	PORTC.OUTSET=0x08;

	// Communication mode: Asynchronous USART
	// Data bits: 8
	// Stop bits: 1
	// Parity: Disabled
	USARTC0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

	// Receive complete interrupt: Low Level
	// Transmit complete interrupt: Low Level
	// Data register empty interrupt: Disabled
	USARTC0.CTRLA=(USARTC0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
	USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_LO_gc | USART_DREINTLVL_OFF_gc;

	// Required Baud rate: 115200
	// Real Baud Rate: 115200.0 (x1 Mode), Error: 0.0 %
	USARTC0.BAUDCTRLA=0x80;
	USARTC0.BAUDCTRLB=((0x09 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x01;

	// Receiver: On
	// Transmitter: On
	// Double transmission speed mode: Off
	// Multi-processor communication mode: Off
	USARTC0.CTRLB=(USARTC0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_RXEN_bm | USART_TXEN_bm;
}

// USARTC0 Receiver buffer
#define RX_BUFFER_SIZE_USARTC0 256
char rx_buffer_usartc0[RX_BUFFER_SIZE_USARTC0];

#if RX_BUFFER_SIZE_USARTC0 <= 256
volatile unsigned char rx_wr_index_usartc0=0,rx_rd_index_usartc0=0;
#else
volatile unsigned int rx_wr_index_usartc0=0,rx_rd_index_usartc0=0;
#endif

#if RX_BUFFER_SIZE_USARTC0 < 256
volatile unsigned char rx_counter_usartc0=0;
#else
volatile unsigned int rx_counter_usartc0=0;
#endif

// This flag is set on USARTC0 Receiver buffer overflow
uint8_t rx_buffer_overflow_usartc0=0;

// USARTC0 Receiver interrupt service routine
ISR(USARTC0_RXC_vect)
{
	unsigned char status;
	char data;

	status=USARTC0.STATUS;
	data=USARTC0.DATA;
	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		rx_buffer_usartc0[rx_wr_index_usartc0++]=data;
		#if RX_BUFFER_SIZE_USARTC0 == 256
		// special case for receiver buffer size=256
		if (++rx_counter_usartc0 == 0) rx_buffer_overflow_usartc0=1;
		#else
		if (rx_wr_index_usartc0 == RX_BUFFER_SIZE_USARTC0) rx_wr_index_usartc0=0;
		if (++rx_counter_usartc0 == RX_BUFFER_SIZE_USARTC0)
		{
			rx_counter_usartc0=0;
			rx_buffer_overflow_usartc0=1;
		}
		#endif
	}
}

// Receive a character from USARTC0
char getchar_usartc0(void)
{
	char data;
	
	while (rx_counter_usartc0==0);
	data=rx_buffer_usartc0[rx_rd_index_usartc0++];
	#if RX_BUFFER_SIZE_USARTC0 != 256
	if (rx_rd_index_usartc0 == RX_BUFFER_SIZE_USARTC0) rx_rd_index_usartc0=0;
	#endif
	asm("cli");
	--rx_counter_usartc0;
	asm("sei");
	return data;
}

// USARTC0 Transmitter buffer
#define TX_BUFFER_SIZE_USARTC0 256
char tx_buffer_usartc0[TX_BUFFER_SIZE_USARTC0];

#if TX_BUFFER_SIZE_USARTC0 <= 256
volatile unsigned char tx_wr_index_usartc0=0,tx_rd_index_usartc0=0;
#else
volatile unsigned int tx_wr_index_usartc0=0,tx_rd_index_usartc0=0;
#endif

#if TX_BUFFER_SIZE_USARTC0 < 256
volatile unsigned char tx_counter_usartc0=0;
#else
volatile unsigned int tx_counter_usartc0=0;
#endif

// USARTC0 Transmitter interrupt service routine
ISR(USARTC0_TXC_vect)
{
	if (tx_counter_usartc0)
	{
		--tx_counter_usartc0;
		USARTC0.DATA=tx_buffer_usartc0[tx_rd_index_usartc0++];
		#if TX_BUFFER_SIZE_USARTC0 != 256
		if (tx_rd_index_usartc0 == TX_BUFFER_SIZE_USARTC0) tx_rd_index_usartc0=0;
		#endif
	}
}

// Write a character to the USARTC0 Transmitter buffer
void putchar_usartc0(char c)
{
	while (tx_counter_usartc0 == TX_BUFFER_SIZE_USARTC0);
	asm("cli");
	if (tx_counter_usartc0 || ((USARTC0.STATUS & USART_DREIF_bm)==0))
	{
		tx_buffer_usartc0[tx_wr_index_usartc0++]=c;
		#if TX_BUFFER_SIZE_USARTC0 != 256
		if (tx_wr_index_usartc0 == TX_BUFFER_SIZE_USARTC0) tx_wr_index_usartc0=0;
		#endif
		++tx_counter_usartc0;
	} else {
		USARTC0.DATA = c;
	}
	asm("sei");
}

// USARTE0 initialization
void usarte0_init(void)
{
	// Note: The correct PORTE direction for the RxD, TxD and XCK signals
	// is configured in the ports_init function.

	// Transmitter is enabled
	// Set TxD=1
	PORTE.OUTSET=0x08;

	// Communication mode: Asynchronous USART
	// Data bits: 8
	// Stop bits: 1
	// Parity: Disabled
	USARTE0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;

	// Receive complete interrupt: Low Level
	// Transmit complete interrupt: Low Level
	// Data register empty interrupt: Disabled
	USARTE0.CTRLA=(USARTE0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
	USART_RXCINTLVL_LO_gc | USART_TXCINTLVL_LO_gc | USART_DREINTLVL_OFF_gc;

	// Required Baud rate: 115200
	// Real Baud Rate: 115200.0 (x1 Mode), Error: 0.0 %
	USARTE0.BAUDCTRLA=0x80;
	USARTE0.BAUDCTRLB=((0x09 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x01;

	// Receiver: On
	// Transmitter: On
	// Double transmission speed mode: Off
	// Multi-processor communication mode: Off
	USARTE0.CTRLB=(USARTE0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_RXEN_bm | USART_TXEN_bm;
}

// USARTE0 Receiver buffer
#define RX_BUFFER_SIZE_USARTE0 256
char rx_buffer_usarte0[RX_BUFFER_SIZE_USARTE0];

#if RX_BUFFER_SIZE_USARTE0 <= 256
volatile unsigned char rx_wr_index_usarte0=0,rx_rd_index_usarte0=0;
#else
volatile unsigned int rx_wr_index_usarte0=0,rx_rd_index_usarte0=0;
#endif

#if RX_BUFFER_SIZE_USARTE0 < 256
volatile unsigned char rx_counter_usarte0=0;
#else
volatile unsigned int rx_counter_usarte0=0;
#endif

// This flag is set on USARTE0 Receiver buffer overflow
uint8_t rx_buffer_overflow_usarte0=0;

// USARTE0 Receiver interrupt service routine
ISR(USARTE0_RXC_vect)
{
	unsigned char status;
	char data;

	status=USARTE0.STATUS;
	data=USARTE0.DATA;
	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		rx_buffer_usarte0[rx_wr_index_usarte0++]=data;
		#if RX_BUFFER_SIZE_USARTE0 == 256
		// special case for receiver buffer size=256
		if (++rx_counter_usarte0 == 0) rx_buffer_overflow_usarte0=1;
		#else
		if (rx_wr_index_usarte0 == RX_BUFFER_SIZE_USARTE0) rx_wr_index_usarte0=0;
		if (++rx_counter_usarte0 == RX_BUFFER_SIZE_USARTE0)
		{
			rx_counter_usarte0=0;
			rx_buffer_overflow_usarte0=1;
		}
		#endif
	}
}

// Receive a character from USARTE0
char getchar_usarte0(void)
{
	char data;

	while (rx_counter_usarte0==0);
	data=rx_buffer_usarte0[rx_rd_index_usarte0++];
	#if RX_BUFFER_SIZE_USARTE0 != 256
	if (rx_rd_index_usarte0 == RX_BUFFER_SIZE_USARTE0) rx_rd_index_usarte0=0;
	#endif
	asm("cli");
	--rx_counter_usarte0;
	asm("sei");
	return data;
}

// USARTE0 Transmitter buffer
#define TX_BUFFER_SIZE_USARTE0 256
char tx_buffer_usarte0[TX_BUFFER_SIZE_USARTE0];

#if TX_BUFFER_SIZE_USARTE0 <= 256
volatile unsigned char tx_wr_index_usarte0=0,tx_rd_index_usarte0=0;
#else
volatile unsigned int tx_wr_index_usarte0=0,tx_rd_index_usarte0=0;
#endif

#if TX_BUFFER_SIZE_USARTE0 < 256
volatile unsigned char tx_counter_usarte0=0;
#else
volatile unsigned int tx_counter_usarte0=0;
#endif

// USARTE0 Transmitter interrupt service routine
ISR(USARTE0_TXC_vect)
{
	if (tx_counter_usarte0)
	{
		--tx_counter_usarte0;
		USARTE0.DATA=tx_buffer_usarte0[tx_rd_index_usarte0++];
		#if TX_BUFFER_SIZE_USARTE0 != 256
		if (tx_rd_index_usarte0 == TX_BUFFER_SIZE_USARTE0) tx_rd_index_usarte0=0;
		#endif
	}
}

// Write a character to the USARTE0 Transmitter buffer
void putchar_usarte0(char c)
{
	while (tx_counter_usarte0 == TX_BUFFER_SIZE_USARTE0);
	asm("cli");
	if (tx_counter_usarte0 || ((USARTE0.STATUS & USART_DREIF_bm)==0))
	{
		tx_buffer_usarte0[tx_wr_index_usarte0++]=c;
		#if TX_BUFFER_SIZE_USARTE0 != 256
		if (tx_wr_index_usarte0 == TX_BUFFER_SIZE_USARTE0) tx_wr_index_usarte0=0;
		#endif
		++tx_counter_usarte0;
	} else {
		USARTE0.DATA = c;
	}
	asm("sei");
}

void gsm_init() {
	PORTD.OUT=0x03;
}

int main(void)
{
	// Declare your local variables here
	unsigned char n;

	// Check the reset source
	n=RST.STATUS;
	if (n & RST_PORF_bm)
	{
		// Power on reset

	}
	else if (n & RST_EXTRF_bm)
	{
		// External reset

	}
	else if (n & RST_BORF_bm)
	{
		// Brown out reset

	}
	else if (n & RST_WDRF_bm)
	{
		// Watchdog reset

	}
	else if (n & RST_PDIRF_bm)
	{
		// Program and debug interface reset

	}
	else if (n & RST_SRF_bm)
	{
		// Software reset

	}
	// Clear the reset flag
	RST.STATUS=n;

	// Interrupt system initialization
	// Make sure the interrupts are disabled
	asm("cli");
	// Low level interrupt: On
	// Round-robin scheduling for low level interrupt: On
	// Medium level interrupt: Off
	// High level interrupt: Off
	// The interrupt vectors will be placed at the start of the Application FLASH section
	n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
	PMIC_LOLVLEN_bm | PMIC_RREN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=n;
	// Set the default priority for round-robin scheduling
	PMIC.INTPRI=0x00;

	// System clocks initialization
	system_clocks_init();

	// Event system initialization
	event_system_init();

	// Ports initialization
	ports_init();

	// Virtual Ports initialization
	vports_init();
	
	// GSM initialization
	gsm_init();

	// USARTC0 initialization
	usartc0_init();

	// USARTE0 initialization
	usarte0_init();

	// Globally enable interrupts
	asm("sei");
	
	uint8_t ok = 0;
	
	while (1) {
		putchar_usartc0('A');
		putchar_usartc0('T');
		putchar_usartc0('\r');
		
		char c = getchar_usartc0();
		
		if (c == 'O') ok = 1;
		if (c == 'K' && ok == 1) break;
	}
	
	putchar_usarte0('O');
	putchar_usarte0('K');
	putchar_usarte0('\r');
	
	led_state(1);
	
	putchar_usartc0('A');
	putchar_usartc0('T');
	putchar_usartc0('I');
	//putchar_usartc0('C');
	//putchar_usartc0('P');
	//putchar_usartc0('I');
	//putchar_usartc0('N');
	//putchar_usartc0('?');
	putchar_usartc0('\r');

	while (1)
	{
		putchar_usarte0(getchar_usartc0());
	}
}