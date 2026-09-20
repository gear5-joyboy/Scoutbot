#define F_CPU 1000000UL // 1 MHz Internal Oscillator
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// ============================================================
// MOTOR DIRECTION CONTROL PINS (PORTB)
// ============================================================
#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

// ============================================================
// PWM OUTPUT PINS (PORTD - Physical Pins 18 & 19)
// ============================================================
#define ENB PD4 // Pin 18 (OC1B)
#define ENA PD5 // Pin 19 (OC1A)

// ============================================================
// ULTRASONIC SENSOR PINS
// TRIG -> PC0 (Physical Pin 22)
// ECHO -> PD2 (Physical Pin 16, INT0)  - must be interrupt-capable
// ============================================================
#define TRIG_PIN PC0

#define OBSTACLE_THRESHOLD_CM   15  // distance below this counts as "obstacle"
#define OBSTACLE_COOLDOWN_TICKS 30  // Timer2 ticks (~100ms each) => ~3s cooldown

// Global Speed Variable (Default to 100% speed = 255)
uint8_t current_speed = 255;

// ============================================================
// PWM INIT — Timer1, Fast PWM 8-bit mode (TOP = 255)
// ============================================================
void PWM_init()
{
	DDRD |= (1 << PD4) | (1 << PD5);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

	OCR1A = current_speed; // Pin 19 (ENA)
	OCR1B = current_speed; // Pin 18 (ENB)
}

void set_speed(uint8_t speed_val)
{
	current_speed = speed_val;
	OCR1A = current_speed;
	OCR1B = current_speed;
}

// ============================================================
// DIRECTION LOGIC
//
// Each function briefly de-energizes all four direction pins
// ("break-before-make") before setting the new direction. This
// matters because an instant flip straight from one direction's
// bit pattern to the opposite one can, on many dual H-bridge
// driver boards (L298N/L293D-style), cause a brief shoot-through
// current spike on one channel. That spike can trip that
// channel's protection or just glitch its output, which shows up
// exactly like "two wheels stop, the other two keep spinning" -
// only the channel that got the bad transition misbehaves, the
// other reverses fine.
// ============================================================
void stopMotors()
{
	PORTB &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
}

void forward()
{
	stopMotors();
	_delay_ms(30); // let both channels fully de-energize first
	PORTB = (PORTB & ~((1 << IN2) | (1 << IN4))) | (1 << IN1) | (1 << IN3);
}

void backward()
{
	stopMotors();
	_delay_ms(30);
	PORTB = (PORTB & ~((1 << IN1) | (1 << IN3))) | (1 << IN2) | (1 << IN4);
}

void left()
{
	stopMotors();
	_delay_ms(30);
	PORTB = (PORTB & ~((1 << IN1) | (1 << IN4))) | (1 << IN2) | (1 << IN3);
}

void right()
{
	stopMotors();
	_delay_ms(30);
	PORTB = (PORTB & ~((1 << IN2) | (1 << IN3))) | (1 << IN1) | (1 << IN4);
}

// ============================================================
// MINIMAL UART DRIVER (to/from ESP32-CAM)
// ============================================================
void UART_init()
{
	UBRRH = 0;
	UBRRL = 12; // 4800 baud @ 1MHz
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

uint8_t UART_available()
{
	return (UCSRA & (1 << RXC));
}

char UART_receive()
{
	while (!(UCSRA & (1 << RXC)));
	return UDR;
}

void UART_send(char data)
{
	while (!(UCSRA & (1 << UDRE)));
	UDR = data;
}

// ============================================================
// TIMER0 — free-running microsecond time base, used only to
// timestamp the ECHO pulse edges inside the INT0 ISR.
// Prescaler = 8 => 1 tick = 8us. 8-bit timer extended to 32-bit
// via an overflow counter.
// ============================================================
volatile uint32_t timer0_overflow_count = 0;

ISR(TIMER0_OVF_vect)
{
	timer0_overflow_count++;
}

void Timer0_init()
{
	TCCR0 = (1 << CS01);   // Normal mode, prescaler = 8
	TIMSK |= (1 << TOIE0); // enable overflow interrupt
}

uint32_t get_time_us()
{
	uint32_t ovf;
	uint8_t cnt;

	uint8_t sreg_save = SREG;
	cli();

	ovf = timer0_overflow_count;
	cnt = TCNT0;

	if ((TIFR & (1 << TOV0)) && (cnt < 255))
	{
		ovf++;
	}

	SREG = sreg_save;

	return ((ovf << 8) | cnt) * 8UL; // ticks -> microseconds
}

// ============================================================
// TIMER2 — periodic ~100ms interrupt.
// This ISR IS the "main loop" for obstacle detection: it fires
// the ultrasonic trigger pulse and ticks down the obstacle
// cooldown, entirely on its own, with zero involvement from
// the code in main().
//
// CTC mode, prescaler = 1024 => tick = 1.024ms
// OCR2 = 97  =>  period ~= 99.4ms
// ============================================================
volatile uint8_t obstacle_cooldown = 0;

void Ultrasonic_trigger()
{
	PORTC &= ~(1 << TRIG_PIN);
	_delay_us(2);
	PORTC |= (1 << TRIG_PIN);
	_delay_us(10);
	PORTC &= ~(1 << TRIG_PIN);
}

ISR(TIMER2_COMP_vect)
{
	Ultrasonic_trigger();

	if (obstacle_cooldown > 0)
	{
		obstacle_cooldown--;
	}
}

void Timer2_init()
{
	TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // CTC, prescaler 1024
	OCR2 = 97;
	TIMSK |= (1 << OCIE2);
}

// ============================================================
// ULTRASONIC SENSOR — fully interrupt-driven measurement.
// INT0 (PD2) fires on both edges of ECHO:
//   rising  -> record start time
//   falling -> compute pulse width, convert to distance, and if
//              it's below the threshold (and not in cooldown),
//              send 'O' to the ESP32-CAM right here, in the ISR.
//
// Nothing in main() is involved in any of this.
// ============================================================
volatile uint32_t echo_start_time = 0;

ISR(INT0_vect)
{
	if (PIND & (1 << PD2))
	{
		// Rising edge: echo just went HIGH
		echo_start_time = get_time_us();
	}
	else
	{
		// Falling edge: echo just went LOW -> pulse finished
		uint32_t pulse_us = get_time_us() - echo_start_time;
		uint16_t distance_cm = (uint16_t)(pulse_us / 58);

		if (distance_cm > 0 && distance_cm < OBSTACLE_THRESHOLD_CM)
		{
			stopMotors(); // safety first: halt immediately regardless of cooldown

			if (obstacle_cooldown == 0)
			{
				UART_send('O'); // tell ESP32-CAM: obstacle seen, take a photo
				obstacle_cooldown = OBSTACLE_COOLDOWN_TICKS;
			}
		}
	}
}

void Ultrasonic_init()
{
	DDRC |= (1 << TRIG_PIN);  // TRIG = output
	PORTC &= ~(1 << TRIG_PIN);

	DDRD &= ~(1 << PD2);      // ECHO (PD2/INT0) = input

	MCUCR |= (1 << ISC00);    // ISC01=0, ISC00=1 -> interrupt on any logical change
	MCUCR &= ~(1 << ISC01);

	GICR |= (1 << INT0);      // enable INT0
}

// ============================================================
// MAIN — now only handles incoming direction/speed commands.
// Obstacle detection runs entirely in TIMER2_COMP_vect + INT0_vect.
// ============================================================
int main()
{
	DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

	stopMotors();
	PWM_init();
	UART_init();
	Timer0_init();
	Timer2_init();
	Ultrasonic_init();

	sei(); // enable global interrupts

	_delay_ms(500);

	// Default movement: start driving forward as soon as power is applied.
	// The robot will keep going forward until a command ('S', 'B', 'L', 'R')
	// arrives over UART and overrides it.
	forward();

	while (1)
	{
		if (UART_available())
		{
			char cmd = UART_receive();

			switch (cmd)
			{
				// Direction Commands
				case 'F': forward(); break;
				case 'B': backward(); break;
				case 'L': left(); break;
				case 'R': right(); break;
				case 'S': stopMotors(); break;

				// Speed Preset Commands
				case '1': set_speed(51);  break; // 20%
				case '2': set_speed(102); break; // 40%
				case '3': set_speed(153); break; // 60%
				case '4': set_speed(204); break; // 80%
				case '5': set_speed(255); break; // 100%
			}
		}
	}
}