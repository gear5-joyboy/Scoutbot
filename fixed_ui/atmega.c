#define F_CPU 1000000UL // 1 MHz Internal Oscillator
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

// ============================================================
// MOTOR DIRECTION CONTROL PINS (PORTB)
// ============================================================
#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB4

// ============================================================
// PWM OUTPUT PINS (PORTD - Physical Pins 18 & 19)
// ============================================================
#define ENB PD4 // Pin 18 (OC1B)
#define ENA PD5 // Pin 19 (OC1A)

// ============================================================
// ULTRASONIC SENSOR PINS
// TRIG -> PC0 (Physical Pin 22)
// ECHO -> PD2 (Physical Pin 16, INT0)  - interrupt-capable
// ============================================================
#define TRIG_PIN PC0

// A slightly larger detection envelope plus dynamic braking prevents
// the car from coasting into an obstacle at higher PWM settings.
#define OBSTACLE_THRESHOLD_CM   45
#define OBSTACLE_COOLDOWN_TICKS 30  // ~3 seconds at 100 ms/tick

// Automatic obstacle response timing.
// 20 timer ticks ~= 2 seconds.
#define OBSTACLE_WAIT_TICKS     20
#define OBSTACLE_REVERSE_TICKS  20
#define OBSTACLE_REVERSE_SPEED  102 // 40% PWM

// Global Speed Variable (Default to 40% speed = 102)
uint8_t current_speed = 102;

// Saved speed so the automatic obstacle routine does not permanently
// change the user's selected speed.
uint8_t saved_speed_before_obstacle = 102;

// ============================================================
// OBSTACLE RESPONSE STATE
// 0 = normal operation
// 1 = stopped/braking, waiting while ESP32 captures the photo
// 2 = reversing away from obstacle
// ============================================================
#define OBSTACLE_STATE_IDLE     0
#define OBSTACLE_STATE_WAIT     1
#define OBSTACLE_STATE_REVERSE  2

volatile uint8_t obstacle_state = OBSTACLE_STATE_IDLE;
volatile uint8_t obstacle_event = 0;
volatile uint8_t obstacle_action_ticks = 0;
volatile uint8_t obstacle_cooldown = 0;

// ============================================================
// PWM INIT — Timer1, Fast PWM 8-bit mode (TOP = 255)
// OCR1A (pin 19/ENA) drives the LEFT motors' speed
// OCR1B (pin 18/ENB) drives the RIGHT motors' speed
// ============================================================
void PWM_init()
{
	DDRD |= (1 << PD4) | (1 << PD5);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler = 8

	OCR1A = current_speed; // Pin 19 (ENA) - LEFT motors
	OCR1B = current_speed; // Pin 18 (ENB) - RIGHT motors
}

void set_speed(uint8_t speed_val)
{
	current_speed = speed_val;
	OCR1A = current_speed;
	OCR1B = current_speed;
}

// ============================================================
// DIRECTION LOGIC
// ============================================================
// stopMotors(): free/coast stop (both bridge inputs LOW).
// brakeMotors(): dynamic brake (both bridge inputs HIGH). This is
// appropriate for the L298/L293-style H-bridge described in the
// original project comments and gives a much shorter stopping distance.
//
// The automatic obstacle response uses brakeMotors() first. Manual
// STOP keeps the original free/coast behavior.
// ============================================================
void stopMotors()
{
	PORTB &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
}

void brakeMotors()
{
	// Both sides brake electrically instead of simply coasting.
	PORTB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
}

void forward()
{
	stopMotors();
	_delay_ms(30); // break-before-make

	OCR1A = current_speed; // left
	OCR1B = current_speed; // right

	PORTB = (PORTB & ~((1 << IN2) | (1 << IN4))) | (1 << IN1) | (1 << IN3);
}

void backward()
{
	stopMotors();
	_delay_ms(30);

	OCR1A = current_speed; // left
	OCR1B = current_speed; // right

	PORTB = (PORTB & ~((1 << IN1) | (1 << IN3))) | (1 << IN2) | (1 << IN4);
}

void left()
{
	// Curve left: right side full speed, left side stopped/slower.
	stopMotors();
	_delay_ms(30);

	OCR1A = 0;              // left - slower
	OCR1B = current_speed;  // right - full speed

	PORTB = (PORTB & ~((1 << IN2) | (1 << IN4))) | (1 << IN1) | (1 << IN3);
}

void right()
{
	// Curve right: left side full speed, right side stopped/slower.
	stopMotors();
	_delay_ms(30);

	OCR1A = current_speed; // left - full speed
	OCR1B = 0;             // right - slower

	PORTB = (PORTB & ~((1 << IN2) | (1 << IN4))) | (1 << IN1) | (1 << IN3);
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
// This ISR only triggers the ultrasonic pulse, updates timing, and
// decrements the cooldown. The actual obstacle reaction runs in main().
// ============================================================
void Ultrasonic_trigger();

ISR(TIMER2_COMP_vect)
{
	Ultrasonic_trigger();

	if (obstacle_cooldown > 0)
	{
		obstacle_cooldown--;
	}

	if (obstacle_state != OBSTACLE_STATE_IDLE && obstacle_action_ticks < 255)
	{
		obstacle_action_ticks++;
	}
}

void Timer2_init()
{
	TCCR2 = (1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20); // CTC, prescaler 1024
	OCR2 = 97;
	TIMSK |= (1 << OCIE2);
}

// ============================================================
// ULTRASONIC SENSOR — interrupt-driven measurement.
// INT0 (PD2) fires on both edges of ECHO:
//   rising  -> record start time
//   falling -> compute pulse width, convert to distance, and when
//              an obstacle is found, request an automatic response.
//
// IMPORTANT: No long delays and no UART transmission happen inside
// this ISR. The old implementation blocked interrupts for seconds,
// which could interfere with UART and timing and made stopping less
// predictable.
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

		if (distance_cm > 0 &&
		distance_cm < OBSTACLE_THRESHOLD_CM &&
		obstacle_cooldown == 0 &&
		obstacle_state == OBSTACLE_STATE_IDLE &&
		obstacle_event == 0)
		{
			// Immediate dynamic brake: this happens at once, without
			// wasting time transmitting or delaying inside the ISR.
			brakeMotors();

			obstacle_event = 1;
			obstacle_cooldown = OBSTACLE_COOLDOWN_TICKS;
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
// ULTRASONIC TRIGGER — sends a ~10us HIGH pulse on TRIG_PIN to
// kick off a new HC-SR04-style measurement.
//
// This is called from inside ISR(TIMER2_COMP_vect) every ~100ms,
// so it MUST stay short and non-blocking: only a microsecond-scale
// busy-wait, no _delay_ms(), no UART, no long loops.
// ============================================================
void Ultrasonic_trigger()
{
	PORTC |= (1 << TRIG_PIN);
	_delay_us(10);
	PORTC &= ~(1 << TRIG_PIN);
}

// ============================================================
// AUTOMATIC OBSTACLE HANDLER
// ============================================================
// Kept in the normal main loop so interrupts remain available.
// Sequence:
//   1) Immediate brake already applied by INT0 ISR.
//   2) Notify ESP32 to capture a photo.
//   3) Wait ~2 s for the photo/display pipeline.
//   4) Reverse for ~2 s at 40% speed.
//   5) Brake, then stop and restore the user's previous speed preset.
// ============================================================
void start_obstacle_response()
{
	uint8_t sreg_save = SREG;
	cli();

	obstacle_event = 0;
	obstacle_state = OBSTACLE_STATE_WAIT;
	obstacle_action_ticks = 0;

	SREG = sreg_save;

	saved_speed_before_obstacle = current_speed;

	// Apply brake again from main as an explicit state transition.
	brakeMotors();

	// Tell ESP32-CAM to capture the obstacle image.
	UART_send('O');
}

void service_obstacle_response()
{
	if (obstacle_state == OBSTACLE_STATE_WAIT && obstacle_action_ticks >= OBSTACLE_WAIT_TICKS)
	{
		obstacle_action_ticks = 0;

		set_speed(OBSTACLE_REVERSE_SPEED);
		backward();
		obstacle_state = OBSTACLE_STATE_REVERSE;
	}
	else if (obstacle_state == OBSTACLE_STATE_REVERSE && obstacle_action_ticks >= OBSTACLE_REVERSE_TICKS)
	{
		obstacle_action_ticks = 0;

		brakeMotors();
		_delay_ms(100);
		stopMotors();

		set_speed(saved_speed_before_obstacle);
		obstacle_state = OBSTACLE_STATE_IDLE;
	}
}

// ============================================================
// MAIN — handles incoming commands and automatic obstacle response.
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

	while (1)
	{
		if (obstacle_event && obstacle_state == OBSTACLE_STATE_IDLE)
		{
			start_obstacle_response();
		}

		service_obstacle_response();

		if (UART_available())
		{
			char cmd = UART_receive();

			// Manual STOP is always honored. Other movement commands are
			// ignored while the automatic obstacle response is active so a
			// browser click cannot immediately defeat the safety maneuver.
			if (cmd == 'S')
			{
				stopMotors();
				obstacle_state = OBSTACLE_STATE_IDLE;
				obstacle_event = 0;
				obstacle_action_ticks = 0;
			}
			else if (obstacle_state == OBSTACLE_STATE_IDLE)
			{
				switch (cmd)
				{
					// Direction Commands
					case 'F': forward();  break;
					case 'B': backward(); break;
					case 'L': left();     break;
					case 'R': right();    break;

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
}