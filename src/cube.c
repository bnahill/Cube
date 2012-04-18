/* Blink LED example */

#include <msp430.h>
#include <stdint.h>

#define BIT(x) (1 << x)

#define FLASH_LEDS 1

// 2.1
#define BLANK_OUT  P2OUT
#define BLANK_DIR  P2DIR
#define BLANK_MASK BIT(1)

// Pin 1.0
#define LED1_OUT  P1OUT
#define LED1_DIR  P1DIR
#define LED1_MASK BIT(0)

// Pin 1.6
#define LED2_OUT  P1OUT
#define LED2_DIR  P1DIR
#define LED2_MASK BIT(6)

// 2.2
#define XLAT_OUT  P2OUT
#define XLAT_DIR  P2DIR
#define XLAT_MASK BIT(2)

// 1.5
#define SCLK_OUT  P1OUT
#define SCLK_DIR  P1DIR
#define SCLK_MASK BIT(5)
#define SCLK_SEL1 P1SEL
#define SCLK_SEL2 P1SEL2

// 1.7
#define MOSI_OUT  P1OUT
#define MOSI_DIR  P1DIR
#define MOSI_SEL1 P1SEL
#define MOSI_SEL2 P1SEL2
#define MOSI_MASK BIT(7)

// 1.4
#define SMCLK_OUT  P1OUT
#define SMCLK_DIR  P1DIR
#define SMCLK_SEL1 P1SEL
#define SMCLK_SEL2 P1SEL2
#define SMCLK_MASK BIT(4)

static uint8_t data[32];

/*!
 @brief Set a single 12-bit value in the write buffer
 @param index The index from 0 to 15
 @param value The 12-bit value
 
 Discards values at higher address to save computation
 */
void set_value(uint8_t index, uint16_t value){
	// Each pair of addressess will be split between two 8-bit addresses.
	// start is the identifier for the pair
	uint8_t start = (index * 3) >> 1;
	if((index & 1) == 0){
		data[start] = (value & 0x0FF0) >> 4;
		data[start+1] = (value & 0x000F) << 4;
	} else {
		data[start] |= (value & 0x0F00) >> 8;
		data[start+1] = (value & 0x00FF);
	}
}

/*!
 @brief Set a pair of 12-bit values in the write buffer
 @param index The EVEN index of the first value to write
 @param value1 The first 12-bit value to write
 @param value2 The second 12-bit value to write
 */
void set_two_values(uint8_t index, uint16_t value1, uint16_t value2){
	index = (index * 3) >> 1;
	data[index++] = (value1 & 0x0FF0) >> 4;
	data[index++] = ((value1 & 0x000F) << 4) | ((value2 & 0x0F00) >> 8);
	data[index] = (value2 & 0x00FF);
}

void spi_init(void){
	// Master, SPI, 8 bits, Idle low, register on first edge
	UCB0CTL0 = 0b10101001;
	// use SMCLK, reset
	UCB0CTL1 = 0b11000001;

	UCB0BR0 = 0;
	UCB0BR1 = 0;

	// Enable
	UCB0CTL1 &= ~0b00000001;
}

inline void spi_wait(void){
	while(UCB0STAT & 1);
}

inline void spi_write(uint8_t word){
	spi_wait();
	UCB0TXBUF = word;
}

inline void xlat_pulse(void){
	XLAT_OUT |= XLAT_MASK;
	XLAT_OUT &= ~XLAT_MASK;
}

inline void blank(uint8_t val){
	if(val)
		BLANK_OUT |= BLANK_MASK;
	else
		BLANK_OUT &= ~BLANK_MASK;
}

void write_buffer(){
	int i;
	for(i = 0; i < 32; i++){
		spi_write(0xFF);
		//spi_write(data[i]);
	}
	for(i = 0; i < 32; i++){
		spi_write(0x00);
		//spi_write(data[i]);
	}
	spi_wait();
	blank(1);
	xlat_pulse();
	blank(0);
}

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;

	// Set MCLK for 16MHz
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;

	// Set SMCLK for MCLK = 16MHz
	BCSCTL2 = 0b00000110;

	__enable_interrupt();

	blank(1);
	BLANK_DIR |= BLANK_MASK;

	XLAT_OUT &= ~XLAT_MASK;
	XLAT_DIR |= XLAT_MASK;

	SCLK_SEL1 |= SCLK_MASK;
	SCLK_SEL2 |= SCLK_MASK;
	SCLK_OUT &= ~SCLK_MASK;
	SCLK_DIR |= SCLK_MASK;

	MOSI_SEL1 |= MOSI_MASK;
	MOSI_SEL2 |= MOSI_MASK;
	MOSI_OUT &= ~MOSI_MASK;
	MOSI_DIR |= MOSI_MASK;

	SMCLK_SEL1 |= SMCLK_MASK;
	SMCLK_SEL2 &= ~SMCLK_MASK;
	SMCLK_OUT &= ~SMCLK_MASK;
	SMCLK_DIR |= SMCLK_MASK;

	TA0R = 0;
	//TA0CCR0 = 32768 / TICK_PER_S;
	TA0CTL = 0b0000000100010010;
	//TA0CCR1 = PULSE_WIDTH;;
	TA0CCTL1 = 0b0000000000110000;

	spi_init();

	set_value(0, 0xFFF);
	set_value(1, 0x000);
	set_value(2, 0xAFF);
	set_value(3, 0x001);
	set_two_values(0, 0xFFF, 0x000);
	set_two_values(2, 0xAFF, 0x001);

	blank(1);

	write_buffer();

	while(1){
		//spi_write(0xAA);
	}
	
}

#ifdef __MSP430G2231
#define TA0IV_TAIFG TAIV_TAIFG
#define TA0IV_TACCR1 TAIV_TACCR1
__attribute__((interrupt(TIMERA1_VECTOR)))
#else
__attribute__((interrupt(TIMER0_A1_VECTOR)))
#endif
void timer_tick_isr(void){
/*
	switch(TA0IV){
	case TA0IV_TAIFG:
		// Clear that flag
		TA0CTL &= ~0x0001;
		TA0CCTL1 |= CCIE;
		LPM3_EXIT;
		break;
	case TA0IV_TACCR1:
		TICK_OUT = TICK_OUT & ~(TICKP_MASK | TICKN_MASK);
		// Clear that flag
		TA0CCTL1 &= ~CCIE;
		TA0CCTL1 &= ~0x0001;
		break;
	}
*/
}

__attribute__((interrupt(WDT_VECTOR)))
void wdt_isr(void){
	//wdt_tick = 1;
	//	IFG1 &= ~WDTIFG;
}

