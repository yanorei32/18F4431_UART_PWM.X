#include <xc.h>

#define _XTAL_FREQ	8000000

#include "uart.h"

// Oscillator selection bits
//   IRCIO: Internal oscillator block, port function on RA6/RA7
#pragma config OSC		= IRCIO

// Fail-safe clock monitor enable bit
#pragma config FCMEN	= OFF

// Internal external oscillator switchover bit
#pragma config IESO		= OFF

// Power-up Timer Enable bit
//   NOTE: for oscillator stability
#pragma config PWRTEN	= ON

// Brown-out reset enable bits
// Brown-out reset voltage bits
//   45: 4.5V
#pragma config BOREN	= OFF
#pragma config BORV		= 45

// Watchdog timer enable bit
// Watchdog timer window enable bit
#pragma config WDTEN	= OFF
#pragma config WINEN	= OFF

// Low/High side transistors polarity
//   PWM0, PWM2, PWM4, PWM6
//   PWM1, PWM3, PWM5, PWM7
#pragma config LPOL		= LOW
#pragma config HPOL		= LOW

// MCLR pin enable bit
// Low-voltage ISCP enable bit
#pragma config MCLRE	= OFF
#pragma config LVP		= OFF

// Background debugger enable bit
#pragma config DEBUG	= OFF

#define FORWARD 0
#define REVERSE 1

// PWM value hard limit for motor
//   720: Close to 7.2V drive
//   480: 66% of 7.2V drive
#define PWM_VALUE_LIMIT	480

/*
	let c = '';

	for( let i = 0; i < 256; i++ ) {
		if (i % 16 == 0) c += '\n';
		c += `${Math.round(i * 480 / 255)}, `;
	}

	c.length -= 1;
*/

const unsigned short PWM_TABLE[] = {
0, 2, 4, 6, 8, 9, 11, 13, 15, 17, 19, 21, 23, 24, 26, 28,
30, 32, 34, 36, 38, 40, 41, 43, 45, 47, 49, 51, 53, 55, 56, 58,
60, 62, 64, 66, 68, 70, 72, 73, 75, 77, 79, 81, 83, 85, 87, 88,
90, 92, 94, 96, 98, 100, 102, 104, 105, 107, 109, 111, 113, 115, 117, 119,
120, 122, 124, 126, 128, 130, 132, 134, 136, 137, 139, 141, 143, 145, 147, 149,
151, 152, 154, 156, 158, 160, 162, 164, 166, 168, 169, 171, 173, 175, 177, 179,
181, 183, 184, 186, 188, 190, 192, 194, 196, 198, 200, 201, 203, 205, 207, 209,
211, 213, 215, 216, 218, 220, 222, 224, 226, 228, 230, 232, 233, 235, 237, 239,
241, 243, 245, 247, 248, 250, 252, 254, 256, 258, 260, 262, 264, 265, 267, 269,
271, 273, 275, 277, 279, 280, 282, 284, 286, 288, 290, 292, 294, 296, 297, 299,
301, 303, 305, 307, 309, 311, 312, 314, 316, 318, 320, 322, 324, 326, 328, 329,
331, 333, 335, 337, 339, 341, 343, 344, 346, 348, 350, 352, 354, 356, 358, 360,
361, 363, 365, 367, 369, 371, 373, 375, 376, 378, 380, 382, 384, 386, 388, 390,
392, 393, 395, 397, 399, 401, 403, 405, 407, 408, 410, 412, 414, 416, 418, 420,
422, 424, 425, 427, 429, 431, 433, 435, 437, 439, 440, 442, 444, 446, 448, 450,
452, 454, 456, 457, 459, 461, 463, 465, 467, 469, 471, 472, 474, 476, 478, 480
};

volatile unsigned char commandTimeout = 0;

void __interrupt() isr(void) {
	uart_isr();

	if (INTCONbits.T0IF) {
		INTCONbits.T0IF = 0;
		commandTimeout = 1;
	}
}

inline unsigned short min(
	unsigned short x,
	unsigned short y
) {
	return x < y ? x : y;
}

#define DEF_SET_PWM(N, POVD_NEG, POVD_POS) \
	void setPWM##N ( \
		unsigned short velocity, \
		unsigned char direction \
	) { \
		velocity = min(velocity, PWM_VALUE_LIMIT); \
		\
		if (direction == FORWARD) { \
			OVDCONDbits.POVD##POVD_NEG = 0; \
			OVDCONDbits.POVD##POVD_POS = 1; \
		} else { \
			OVDCONDbits.POVD##POVD_NEG = 1; \
			OVDCONDbits.POVD##POVD_POS = 0; \
		} \
		\
		PDC##N##H = velocity >> 8; \
		PDC##N##L = velocity & 0xFF; \
	}

DEF_SET_PWM(0, 0, 1)
DEF_SET_PWM(1, 2, 3)
DEF_SET_PWM(2, 4, 5)
DEF_SET_PWM(3, 6, 7)

inline void shortBrake() {
	// PWM output override control register
	//   1: Output on PWM I/O pin is controlled by the value in
	//      the Duty Cycle register and the PWM time base
	//   0: Output on PWM I/O pin is controlled by the value in
	//      the corresponding POUT bit.
	OVDCONDbits.POVD = 0x00;

	// PWM output state register
	//   1: Output on PWM I/O pin is active
	//      when the corresponding PWM output override bit is cleared
	//   0: Output on PWM I/O pin is inactive
	//      when the corresponding PWM output override bit is cleared
	//   NOTE: 1 (LOW) / 0 (HIGH)
	OVDCONSbits.POUT = 0x00;
}

inline void resetTimer() {
	TMR0H = 0;
	TMR0L = 0;
}

int main() {
	/*\
	|*| Oscillator configuration
	\*/
	
	// Internal oscillator frequency slect bits
	//   0b111: 8MHz source drives clock directly
	OSCCONbits.IRCF = 0b111;

	// System clock select bits
	//   1: Internal oscillator block
	OSCCONbits.SCS1 = 1;


	/*\
	|*| PWM configuration
	\*/

	// PWM output override bits
	//   0: follow POUT (default: 0) value
	OVDCONDbits.POVD = 0;

	// PWM time base period register
	//   (Fosc / 4) / (PTPER + 1) Hz / 7812.5Hz
	//
	//   PTPERL: lower 8 bits
	//   PTPERH: upper 4 bits
	PTPERL = 0xFF;
	PTPERH = 0;

	// PWM output pair mode bits
	//   1: PWM I/O pin pair is in the independent mode
    //        PWM0/PWM1, PWM2/PWM3, PWM4/PWM5, PWM6/PWM7
	PWMCON0bits.PMOD = 0b1111;

	// PWM time base timer enable bit
	//   1: on
	PTCON1bits.PTEN = 1;

	shortBrake();

	// PWM module enable bits
	//   0b101: All PWM I/O pins are enabled for PWM output
	PWMCON0bits.PWMEN = 0b101;


	/*\
	|*| UART configuration
	\*/

	// Global interrupt enable bit
	//   1: Enables all unmasked interrupts
	INTCONbits.GIE	= 1;

	// Peripheral interrupt enable bit
	//   1: Enables all unmasked peripheral iterrupts
	INTCONbits.PEIE	= 1;

	// Set data direction of UART TX/RX
	//   1: input
	TRISCbits.TRISC6 = TRISCbits.TRISC7 = 1;
	uart_on();

	
	/*\
	|*| Timer configuration
	\*/

	// Timer0 clock source slect bit
	//   0: Internal clock (FOSC/4)
	T0CONbits.T0CS = 0;

	// Timer0 prescaler assignment bit
	//   0: Timer0 prescaler is assigned.
	T0CONbits.PSA = 0;

	// Timer0 16-bit control bit
	//   0: Timer0 is configured as a 16-bit timer/counter
	T0CONbits.T016BIT = 0;

	// Timer0 prescaler select bits
	//   0b111: 1:256 Prescale value
	//   0b110: 1:128 Prescale value
	//   0b101: 1:64 Prescale value
	//   0b100: 1:32 Prescale value
	//   0b011: 1:16 Prescale value
	//   0b010: 1:8 Prescale value
	//   0b001: 1:4 Prescale value
	//   0b000: 1:2 Prescale value
	//
	//   command timeout (sec):
	//     1 / (8,000,000 Hz / 4 (FOSC/4) / 65536 (16-bit) / PRESCALER) sec
	//     0b011: 0.524288 sec
	T0CONbits.T0PS = 0b011;

	// Timer0 interrupt
	INTCONbits.T0IE = 1;

	resetTimer();

	// commandPosition
	//   x + f f \n
	//   0 1 2 3 4
	//   y - 0 0 \n
	//   0 1 2 3 4
	//   z - f f \n
	//   0 1 2 3 4
	//   w + 0 0 \n
	//   0 1 2 3 4
	unsigned char commandPosition = 0;
	unsigned char id, direction, velocity;

	while (1) {
		if (!uart_rx_buf_num()) {
			if (commandTimeout) {
				shortBrake();
				commandTimeout = 0;
			}
			continue;
		}

#ifdef DEBUG
		char c = getche();
#else
		char c = getch();
#endif
		
		if (commandPosition == 0) {
			if ( 'w' <= c && c <= 'z' )
				id = c == 'w' ? 3 : c - 'x';

			else
				goto PARSE_ERROR;

		} else if (commandPosition == 1) {
			if (c == '+')
				direction = FORWARD;

			else if (c == '-')
				direction = REVERSE;

			else
				goto PARSE_ERROR;

		} else if (commandPosition == 2) {
			if ('0' <= c && c <= '9')
				velocity = (unsigned char) ( (c - '0') << 4 );

			else if ('a' <= c && c <= 'f')
				velocity = (unsigned char) ( (c - 'a' + 10) << 4 );

			else
				goto PARSE_ERROR;

		} else if (commandPosition == 3) {
			if ('0' <= c && c <= '9')
				velocity |= (c - '0');

			else if ('a' <= c && c <= 'f')
				velocity |= (c - 'a' + 10);

			else
				goto PARSE_ERROR;

		} else if (commandPosition == 4) {
			if (c != '\n')
				goto PARSE_ERROR;
		}

		commandPosition += 1;

		if (commandPosition != 5)
			continue;

		// float pwmValue = PWM_VALUE_LIMIT / 255.0f * velocity;
		// unsigned short pwmValue = PWM_TABLE[velocity];
        unsigned short pwmValue = (unsigned short)(
			(uint24_t) velocity * PWM_VALUE_LIMIT / 255
		);

		if (id == 0)
			setPWM0(pwmValue, direction);

		else if (id == 1)
			setPWM1(pwmValue, direction);

		else if (id == 2)
			setPWM2(pwmValue, direction);

		else if (id == 3)
			setPWM3(pwmValue, direction);

		resetTimer();

PARSE_ERROR:
#ifdef DEBUG
		putch(commandPosition + '0');
#endif
		commandPosition = 0;

	}

	return 0;
}

