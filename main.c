/**
 * Author: Ray Russell Reese III <russ@zerotech.net>
 * 
 * Quadruped - aka Steve v1.0.
 * 
 *
 * Looking down at the robot with the vreg at the top:
 * SERVO #		Description
 * 		 0 		Front Right Hip
 *       1      Front Right Leg
 *       2      Back Right Hip
 *       3      Back Right Leg
 *       4      Back Left Hip
 *       5      Back Left Leg
 *       6      Front Left Hip
 *       7      Front Left Leg
 *
 */

#define DEBUG

#define BAUD 9600

#include <stdio.h>
#include <stdint.h> 
#include <stdbool.h> 
#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h> 
#include <util/setbaud.h>

void uart_putchar(char c, FILE *stream);
char uart_getchar(FILE *stream);
// Utility Macros

// Macro for setting the servo with offset calibration applied.
#define S(I,X) servoSet(I, SERVO_MID + servoCenterOffsets[I] + X)
#define _BV(bit) (1 << (bit))

#define FORWARD 0
#define BACKWARD 1

// LVC: Voltage Divider Resistors (in Ohms)

#define VD_R1 680000.0	
#define VD_R2 100000.0

// Internal voltage reference. Calibrate with a meter.

#define IVREF 1125300L

// Servo related settings.

#define N_SERVOS    8

#define SERVO_MIN    544
#define SERVO_MAX   2400
#define SERVO_MID   (SERVO_MIN + SERVO_MAX) / 2 

// Pulse width frame.
#define SERVO_FRAME 20000 // micros - 50Hz

// Time slot available for each servo. 
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS) 

// For future reference so I don't forget.
#if (SERVO_TIME_DIV < SERVO_MAX + 50) 
#warning "Output fewer servo signals or increase SERVO_FRAME" 
#endif 
#if ((SERVO_TIME_DIV * (F_CPU / 1000000UL)) >= 0xFF00) 
#warning "Output more servo signals or decrease SERVO_FRAME (or use the prescaler)" 
#endif 

#define US2TIMER1(us) ( (us) * (uint16_t)(F_CPU / 1E6) ) 
#define TIMER12US(t1) ( (uint16_t)(F_CPU * 1E6) / (t1) )

volatile bool lvc_cutoff = false;

volatile uint16_t servoTime[N_SERVOS]; 
volatile uint16_t servoUs[N_SERVOS];
volatile uint16_t receiverUs[3] = {0, 0, 0};
volatile uint16_t receiverPwm[3] = {0, 0, 0};
volatile bool receiverLast[3] = {true, true, true};

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

// Bitmasks for the location of the servos on the port.
const static uint8_t servoOutMask[N_SERVOS] = {
	0b00000001,
	0b10000000,
	0b01000000,
	0b00100000,
	0b10000000,
	0b01000000,
	0b00010000,
	0b00001000,
}; 

// Mode register addresses for each servo port.
const static uint8_t servoDdr[N_SERVOS] = {
	(uint8_t) &DDRB,
	(uint8_t) &DDRD,
	(uint8_t) &DDRD,
	(uint8_t) &DDRD,
	(uint8_t) &DDRB,
	(uint8_t) &DDRB,
	(uint8_t) &DDRD,
	(uint8_t) &DDRD	
};

// Port register addresses for each servo.
const static uint8_t servoPort[N_SERVOS] = {
	(uint8_t) &PORTB,
	(uint8_t) &PORTD,
	(uint8_t) &PORTD,
	(uint8_t) &PORTD,
	(uint8_t) &PORTB,
	(uint8_t) &PORTB,
	(uint8_t) &PORTD,
	(uint8_t) &PORTD
};

// These are the offsets used to center the servos when applying SERVO_MID.
const static uint16_t servoCenterOffsets[N_SERVOS] = {
	-200,
	-130,
	10,
	-175,
	-240,
	-125,
	230,
	-100
};

void spiderStand();

// Uses the internal reference voltage to read the true Vcc of the chip.
// This can be used to get a more accurate reading via the ADC.
uint16_t readVcc() {
	ADMUX = 0;
	ADMUX |= _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	// Settle Vref
	_delay_ms(2);

	// Adjust ADC prescaler for clock speed.
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);

	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADSC);
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t l = ADCL;
	uint8_t h = ADCH;

	uint16_t r = (h << 8) | l;
	return IVREF / r;	
}

uint16_t readBatteryVoltage() {
	ADMUX = 0;
	ADMUX |= _BV(REFS0) | _BV(MUX2); // Select ADC4
	// Settle Vref
	_delay_ms(2);

	// Adjust ADC prescaler for clock speed.
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1);

	ADCSRA |= _BV(ADEN);
	ADCSRA |= _BV(ADSC);
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t l = ADCL;
	uint8_t h = ADCH;

	uint16_t r = (h << 8) | l;

	return r;
}

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
}

char uart_getchar_nb() {
	if (bit_is_set(UCSR0A, RXC0))
		return UDR0;
	return NULL;
}

// Setup the ports used by the servo, and setup the timer for the PWM.
void servoStart(void) { 
	if (lvc_cutoff)
		return;

    int i;
    for (i = 0; i < N_SERVOS; i++) {
    	volatile uint8_t *reg = (volatile uint8_t *)(servoDdr[i]);
    	*reg |= servoOutMask[i];
    }

    // Setupt a first compare match 
    OCR1A = TCNT1 + US2TIMER1(100); 
    // start timer 1 with no prescaler 
    TCCR1B = (1 << CS10);       
    // Enable interrupt 
    TIMSK1 |= (1 << OCIE1A); 
} 

// Set the microsecond pulse width. 
void servoSet(uint8_t servo, uint16_t time) { 
	if (lvc_cutoff)
		return;

	if (time > SERVO_MAX)
		time = SERVO_MAX;
	if (time < SERVO_MIN)
		time = SERVO_MIN;

	servoUs[servo] = time;

    uint16_t ticks = US2TIMER1(time); 
    cli(); 
    servoTime[servo] = ticks; 
    sei(); 
} 

void servoInc(uint8_t servo, uint16_t time) { 
	if (lvc_cutoff)
		return;

	// Ignore when it exceeds the servo max.
	if (servoUs[servo] + time > SERVO_MAX)
		return;

	servoUs[servo] += time;

    uint16_t ticks = US2TIMER1(time);
    cli(); 
    servoTime[servo] = servoTime[servo] + ticks; 
    sei(); 
} 

void servoDec(uint8_t servo, uint16_t time) { 
	if (lvc_cutoff)
		return;

	// Ignore when it exceeds the servo min.
	if (servoUs[servo] - time < SERVO_MIN)
		return;

	servoUs[servo] -= time;

    uint16_t ticks = US2TIMER1(time); 
    cli(); 
    servoTime[servo] = servoTime[servo] - ticks; 
    sei(); 
} 

void wave() {
	// Spider Stand first:

	spiderStand();
	_delay_ms(100);

	// Now center the front right so we have balance.
	S(6, -700);
	S(7, 700);

	S(1, -700);
	_delay_ms(400);

	S(1, 0);
	_delay_ms(400);

	S(1, -700);
	_delay_ms(400);

	S(1, 0);
	_delay_ms(400);
	S(1, -700);
	_delay_ms(400);

	S(1, 0);
	_delay_ms(400);
}

void comeAtMeBro() {
	S(3, 200);
	S(5, 200);
	_delay_ms(20);

	S(2, 0);
	S(4, 0);
	_delay_ms(20);

	S(0, -300);
	S(6, 300);
	_delay_ms(20);

	S(1, 1200);
	S(7, 1200);
}

void layFlatSlow() {
	int done = 0;

	while (!done) {
		int x = 0;
		done = 1;
		for (x = 0; x < N_SERVOS; x++) {
			if (servoUs[x] > (SERVO_MID + servoCenterOffsets[x])) {
				servoDec(x, 1);
				done = 0;
			} else if (servoUs[x] < (SERVO_MID + servoCenterOffsets[x])) {
				servoInc(x, 1);
				done = 0;
			} 
		}
		_delay_ms(5);
	}
}

void layFlat() {
	int i;

	for (i = 0; i < N_SERVOS; i++) {
		S(i, 0);
		_delay_ms(20);
	}
}

void spiderStand() {
	int i;

	for (i = 0; i < N_SERVOS; i += 4) {
		S(i, -300);
		S(i + 2, 300);

		_delay_ms(20);
	}

   	for (i = 0; i < N_SERVOS; i += 2) {   		
   		S(i + 1, 900);
		_delay_ms(20);
   	}
}

void leftIn() {
	S(1, 1200);	
	_delay_ms(100);
	S(0, -700);
	_delay_ms(20);
	S(3, 600);
	_delay_ms(100);
	S(2, 700);
	_delay_ms(20);
}

void leftOut() {
	S(1, 600);
	_delay_ms(100);
	S(0, -100);
	_delay_ms(30);
	S(3, 1200);
	_delay_ms(100);
	S(2, 100);
	_delay_ms(30);
}

void rightIn() {
	S(5, 600);
	_delay_ms(100);
	S(4, -700);
	_delay_ms(30);
	S(7, 1200);
	_delay_ms(100);
	S(6, 700);
	_delay_ms(30);
}

void rightOut() {
	S(5, 1200);
	_delay_ms(100);
	S(4, -100);
	_delay_ms(30);
	S(7, 600);
	_delay_ms(100);
	S(6, 100);
	_delay_ms(30);
}

void leftInRightOut(int direction) {
	// Legs
	if (direction == FORWARD) {
		S(1, 1200);	
		S(5, 1200);
		S(3, 400);
		S(7, 400);
	} else if (direction == BACKWARD) {
		S(5, 400);
		S(1, 400);
		S(7, 1200);
		S(3, 1200);		
	}
	_delay_ms(100);

	// Hips
	S(0, -700);
	S(4, -100);
	S(2, 700);
	S(6, 100);
}

void rightInLeftOut(int direction) {
	// Legs
	if (direction == FORWARD) {
		S(5, 400);
		S(1, 400);
		S(7, 1200);
		S(3, 1200);
	} else if (direction == BACKWARD) {
		S(1, 1200);	
		S(5, 1200);
		S(3, 400);
		S(7, 400);
	}
	_delay_ms(100);
	
	// Hips
	S(4, -700);
	S(0, -100);
	S(6, 700);
	S(2, 100);
}

int main(void) { 
	// Enable status LED.

	uart_init();
    stdout = &uart_output; // Throw stdout via the TX pin on the atmega328p.
    stdin = &uart_input;

	DDRC |= _BV(5);;
	DDRC &= ~_BV(3);

	// Setup Timer0 for PWM reading.

	TCCR0B |= _BV(CS10);
	OCR0A = TCNT0 + 50;
	TIMSK0 |= _BV(OCIE0A);    

	// Setup servos.    
    servoStart(); 

    // Enable interrupts.
    sei();

    // Let the interrupts settle.
    spiderStand();
    // Break so we can remove the programmer.
    _delay_ms(500);

	uint8_t selectedServo = 0;
	uint16_t servoMove = 100;
	uint16_t cal;
	int dir = FORWARD;
	int count = 0;
	double vcc;
	double adc4_p;
	double battvcc;
	bool test_lvc = false;

    for(;;) {
    	// This implements the low voltage cutout warning.

    	double vcc = readVcc() / 1000.0;
    	double adc4_p = readBatteryVoltage();
    	double battvcc = adc4_p * (vcc / 1023.0) * ((VD_R1 + VD_R2) / VD_R2);

		printf("BATT VCC: %0.2f, VREG VCC: %0.2f\n", battvcc, vcc);

    	// Low voltage detection
    	if (battvcc < 6.5 || test_lvc) {
    		printf("Battery voltage dropped to %0.2f\n", battvcc);
    		// Test it once more under delay conditions.
    		_delay_ms(300);
	    	adc4_p = readBatteryVoltage();
    		battvcc = adc4_p * (vcc / 1023.0) * ((VD_R1 + VD_R2) / VD_R2);

    		if (battvcc < 6.5 || test_lvc) {
	    		printf("-> Second reading: %0.2f\n", battvcc);
	    		PORTC &= ~_BV(5);
    			cli();
    			lvc_cutoff = true;
    		}
    	} else {
    		PORTC |= _BV(5);
    	}

    	uint8_t in = uart_getchar_nb();

    	if (in != NULL) {
	    	switch (in) {
	    		case '?':
	    			// Show a brief list of commands
	    			printf("Commands:\n");
	    			printf(" z - Toggle direction used in manual walking\n");
	    			printf(" c - Show current calibration setting (with adjustments)\n");
	    			printf(" [ - Decrease servo step by 10\n");
	    			printf(" ] - Increase servo step by 10\n");
	    			printf(" v - Show voltage information\n");
	    			printf(" k - Left In/Right Out walk phase\n");
	    			printf(" l - Right In/Left Out walk phase\n");
	    			printf(" F - Quickly center all servos\n");
	    			printf(" f - Slowly center all servos\n");
	    			printf(" = - Increment servo pulse by step value\n");
	    			printf(" - - Decrement servo pulse by step value\n");
	    			printf(" b - Activate 'Come at me bro' stance\n");
	    			printf(" s - Activate 'Spider Stand' stance\n");
	    			printf(" p - Show receiver channel signals\n");
	    			printf(" X - Disable interupts\n");
	    			printf(" x - Enable interrupts\n");
	    			printf(" T - Test low voltage cutoff\n");
	    			printf(" t - Stop testing low voltage cutoff\n");
	    			printf(" # - Select 1-8 selects servo 0-7\n");
	    			break;
	    		case 'T':
	    			test_lvc = true;
	    			break;
	    		case 't':
	    			test_lvc = false;
	    			lvc_cutoff = false;
	    			sei();
	    			break;
	    		case 'z':
	    			dir = !dir;
	    			break;
	    		case 'c': // Gets the calibration value for the currently selected servo less with adjustments factored in.
	    			cal = servoUs[selectedServo] - SERVO_MID;
	    			printf("Servo center calibration value for %i: %i\n", selectedServo, cal);
	    			break;
	    		case '[':
	    			servoMove -= 10;
	    			printf("Servo step reduced to %i\n", servoMove);
	    			break;
	    		case ']':
	    			servoMove += 10;
	    			printf("Servo step increased to %i\n", servoMove);
					break;
	    		case 'v':
	    			printf("BATT VCC: %0.2f, VREG VCC: %0.2f\n", battvcc, vcc);
	    			break;
	    		case 'k': // Simulate first phase of walk.
	    			leftInRightOut(dir);
	    			break;
	    		case 'l': // Simulate last phase of walk.
	    			rightInLeftOut(dir);
	    			break;
	    		case 'F':
	    			layFlat();
	    			break;
	    		case 'f': // Lay flat. Also good for calibrations.
	    			layFlatSlow();
	    			break;
	    		case '=': // Increment the servo count by 100
	    			printf("Servo %i decremented by %i\n", selectedServo, servoMove);
	    			servoInc(selectedServo, servoMove);
	    			break;
	    		case '-': // Decrement the servo count by 100
	    			printf("Servo %i incremented by %i\n", selectedServo, servoMove);
	    			putchar(1);
	    			servoDec(selectedServo, servoMove);
	    			break;
	    		case 's': // Stand in "spider mode"
	    			printf("Spider Stand selected.\n");
	    			spiderStand();
	    			break;
	    		case 'b': // Stand in "come at me bro mode"
	    			printf("Come at me Bro selected.\n");
	    			comeAtMeBro();
	    			break;
	    		case 'p':
	    			printf("Receiver Signal 0: %i\n", receiverPwm[0]);
	    			break;
	    		case 'X':
	    			printf("Interrupts disabled\n");
	    			cli();
	    			break;
	    		case 'x':
	    			printf("Interrupts enabled\n");
	    			sei();
	    			break;
	    		default:
	    			if ((in > 48) && (in <= 56)) { // Select a servo number.
	    				printf("Servo # %i selected.\n", in - 49);
	    				selectedServo = in - 49;
	    			} else {
	    				printf("%i", in);
	    			}

	    	}
	    }

	    if (receiverPwm[0] > 4000) {
	    	/**leftInRightOut(FORWARD);
			_delay_ms(150);
	    	rightInLeftOut(FORWARD);
			_delay_ms(150);*/
	    } else if (receiverPwm[0] < 3600 && receiverPwm[0] != 0) {
	    	/**leftInRightOut(BACKWARD);
			_delay_ms(150);
	    	rightInLeftOut(BACKWARD);
			_delay_ms(150);*/
	    } else {
	    	// Do nothing;
	    }

    }
} 


ISR(TIMER0_COMPA_vect) {
	static uint16_t lastTime = 0;
	static bool lastState = false;

	if (PINC & _BV(3)) {
		if (lastState) {
			lastTime += TCNT0;
			return;
		} else {
			lastState = true;
			lastTime = TCNT0;
		}
	} else {
		if (!lastState) {
			return;
		} else {
			lastState = false;
			receiverPwm[0] = lastTime;
			lastTime = 0;
		}
	}
}

// ISR Timer for controlling PWM signal.

ISR(TIMER1_COMPA_vect) { 
    static uint16_t nextStart; 
    static uint8_t servo; 
    static bool outputHigh = true; 
    uint16_t currentTime = OCR1A; 
    uint8_t mask = servoOutMask[servo]; 
    volatile uint8_t *reg = (volatile uint8_t *)(servoPort[servo]);

    if (outputHigh) { 
        *reg |= mask; 

        // Set the end time for the servo pulse 
        OCR1A = currentTime + servoTime[servo]; 
        nextStart = currentTime + US2TIMER1(SERVO_TIME_DIV); 
    } else { 
        *reg &= ~mask; 
        if (++servo == N_SERVOS) { 
            servo = 0; 
        } 
        OCR1A = nextStart; 
    } 
    outputHigh = !outputHigh; 
}