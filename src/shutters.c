#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// LED RGB ze wspólną anodą
#define LED_DDR DDRA
#define LED_PORT PORTA
#define LED_R 0
#define LED_G 1
#define LED_B 2

#define R_ON LED_PORT &= ~(1 << LED_R);
#define R_OFF LED_PORT |= (1 << LED_R);
#define G_ON LED_PORT &= ~(1 << LED_G);
#define G_OFF LED_PORT |= (1 << LED_G);
#define B_ON LED_PORT &= ~(1 << LED_B);
#define B_OFF LED_PORT |= (1 << LED_B);

// Rolety
#define SHUTTERS_DDR DDRD
#define SHUTTERS_PORT PORTD
#define SHUTTERS_UP_PIN 2
#define SHUTTERS_DOWN_PIN 3

#define SHUTTERS_INACTIVE_UP (SHUTTERS_PORT &= ~(1<<SHUTTERS_UP_PIN));
#define SHUTTERS_ACTIVE_UP (SHUTTERS_PORT |= (1<<SHUTTERS_UP_PIN));

#define SHUTTERS_INACTIVE_DOWN (SHUTTERS_PORT &= ~(1<<SHUTTERS_DOWN_PIN));
#define SHUTTERS_ACTIVE_DOWN (SHUTTERS_PORT |= (1<<SHUTTERS_DOWN_PIN));

#define SHUTTERS_DIR_DOWN 0
#define SHUTTERS_DIR_UP 1

unsigned char isShutterActive = 0; // bool = usnigned char
unsigned char shutterDirection = SHUTTERS_DIR_DOWN;

/* BTM-222
 * 19200 bps; no parity; 1 stop bit
 */
#define SPEED_BAUD 19200UL
#define UBRR_value (( (F_CPU)/(16UL*SPEED_BAUD) ) - 1)

#define DELAY 100 // ms


// Prototypy funkcji
void blinkRGB(void);
void init_usart(unsigned int);
uint8_t USARTReceiveByte(void);
void initShutters();
void init_led(void);


int main(void)
{
	init_led();
	initShutters();
	init_usart(UBRR_value);

	for (uint8_t i = 0; i < 3; i++) {
		blinkRGB();
	}

	sei();

  while(1) {

  }

	return 0;
}

void blinkRGB(void) {
  R_ON
	_delay_ms(DELAY);
	R_OFF
	_delay_ms(DELAY);
	G_ON
	_delay_ms(DELAY);
	G_OFF
	_delay_ms(DELAY);
	B_ON
	_delay_ms(DELAY);
	B_OFF
	_delay_ms(DELAY);
}

void init_led(void) {
	// ustaw piny diody LED jako wyjście
	LED_DDR |= (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
	// wyłącz wszystkie kolory diody LED
  R_OFF
	G_OFF
	B_OFF
}

void init_usart(unsigned int ubrr){
	// ustaw prędkość transmisji
	UBRRH = (unsigned char) (ubrr>>8);
	UBRRL = (unsigned char) ubrr;
	// włącz odbiór danych, transmisję danych i
	// przerwanie przy odbiorze znaku USART
	UCSRB = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
	// ramka 8-bitowa
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
}

void initShutters() {
	// piny rolet jako wyjście
	SHUTTERS_DDR |= (1<<SHUTTERS_DOWN_PIN)|(1<<SHUTTERS_UP_PIN);
	SHUTTERS_INACTIVE_DOWN
	SHUTTERS_INACTIVE_UP
}

uint8_t USARTReceiveByte(void)
{
	while ( !(UCSRA & (1<<RXC)) )
		continue;
	return UDR;
}

// Procedura obsługi przerwania odbioru znaku USART
ISR(USART_RXC_vect) {
	unsigned char input;
	input = UDR;
	// UDR = input; // echo

	switch (input) {
		case 'r':
		// case 'R':
			if (isShutterActive == 1) {
			SHUTTERS_INACTIVE_DOWN
			SHUTTERS_INACTIVE_UP
			isShutterActive = 0;
			} else {
				SHUTTERS_INACTIVE_UP
				SHUTTERS_ACTIVE_DOWN
				isShutterActive = 1;
			}
			break;
		case 'g':
		// case 'G':
			if (isShutterActive == 1) {
			SHUTTERS_INACTIVE_DOWN
			SHUTTERS_INACTIVE_UP
			isShutterActive = 0;
			} else {
				SHUTTERS_INACTIVE_DOWN
				SHUTTERS_ACTIVE_UP
				isShutterActive = 1;
			}
			break;
		default:
			break;
	}
}