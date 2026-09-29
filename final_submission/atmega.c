// Existing hardware configuration: internal 1 MHz, unchanged pins and baud.
#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
#define OBSTACLE_STATE_BRAKE    3

volatile uint8_t obstacle_state = OBSTACLE_STATE_IDLE;
volatile uint8_t obstacle_event = 0;
volatile uint8_t obstacle_action_ticks = 0;
volatile uint8_t obstacle_cooldown = 0;
volatile char pending_drive = 0;
volatile uint8_t stop_requested = 0;
uint32_t drive_after_us = 0;
uint32_t brake_until_us = 0;
uint32_t get_time_us(void);

// ============================================================
// PWM INIT — Timer1, Fast PWM 8-bit mode (TOP = 255), /8 at 1 MHz
// OCR1A (pin 19/ENA) drives the LEFT motors' speed
// OCR1B (pin 18/ENB) drives the RIGHT motors' speed
// ============================================================
void PWM_init()
{
	DDRD |= (1 << PD4) | (1 << PD5);

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // Original /8 motor PWM

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

// Preserve the 30 ms break-before-make interval without pausing GPS input.
void request_drive(char direction)
{
    uint32_t deadline = get_time_us() + 30000UL;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (!obstacle_event && !stop_requested)
        {
            stopMotors();
            drive_after_us = deadline;
            pending_drive = direction;
        }
    }
}
void forward()  { request_drive('F'); }
void backward() { request_drive('B'); }
void left()     { request_drive('L'); }
void right()    { request_drive('R'); }

void service_drive()
{
    if (!pending_drive || (int32_t)(get_time_us() - drive_after_us) < 0) return;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (!obstacle_event && !stop_requested && pending_drive &&
            (obstacle_state == OBSTACLE_STATE_IDLE || obstacle_state == OBSTACLE_STATE_REVERSE))
        {
            char direction = pending_drive;
            pending_drive = 0;
            OCR1A = direction == 'L' ? 0 : current_speed;
            OCR1B = direction == 'R' ? 0 : current_speed;
            if (direction == 'B')
                PORTB = (PORTB & ~((1 << IN1) | (1 << IN3))) | (1 << IN2) | (1 << IN4);
            else
                PORTB = (PORTB & ~((1 << IN2) | (1 << IN4))) | (1 << IN1) | (1 << IN3);
            if (obstacle_state == OBSTACLE_STATE_REVERSE) obstacle_action_ticks = 0;
        }
    }
}

// ============================================================
// MINIMAL UART DRIVER (to/from ESP32-CAM)
// ============================================================
#define TX_SIZE 128
#define RX_SIZE 16
volatile char tx_queue[TX_SIZE], rx_queue[RX_SIZE];
volatile uint8_t tx_head = 0, tx_tail = 0, rx_head = 0, rx_tail = 0;

// Transmit between GPS samples; no periodic TX interrupt can move a sample.
static inline __attribute__((always_inline)) void UART_tx_poll(void)
{
    if ((UCSRA & (1 << UDRE)) && tx_tail != tx_head)
    {
        UDR = tx_queue[tx_tail];
        tx_tail = (tx_tail + 1) & (TX_SIZE - 1);
    }
}

ISR(USART_RXC_vect)
{
    uint8_t status = UCSRA;
    char c = UDR;
    if (status & ((1 << FE) | (1 << DOR) | (1 << PE))) return;
    if (c == 'S')
    {
        // STOP is acted on immediately, even while a GPS sentence is parsed.
        PORTB &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
        pending_drive = 0;
        stop_requested = 1;
        rx_tail = rx_head; // Discard commands that preceded this STOP.
        return;
    }
    uint8_t next = (rx_head + 1) & (RX_SIZE - 1);
    if (next != rx_tail) { rx_queue[rx_head] = c; rx_head = next; }
}

void UART_init()
{
    UCSRA = 0;
    UBRRH = 0;
    UBRRL = 12; // Original 4800 baud @ 1 MHz, normal speed
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}
uint8_t UART_available() { return rx_head != rx_tail; }
char UART_receive()
{
    char c = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (rx_tail != rx_head)
        {
            c = rx_queue[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_SIZE - 1);
        }
    }
    return c;
}
void UART_send(char data)
{
    uint8_t next = (tx_head + 1) & (TX_SIZE - 1);
    // Normal T + G + O traffic fits this queue. Interrupts must be enabled
    // here; this function is only called from the main loop.
    while (next == tx_tail) { UART_tx_poll(); }
    tx_queue[tx_head] = data;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        tx_head = next;
    }
}
void UART_send_string(const char *s) { while (*s) UART_send(*s++); }

// ============================================================
// TIMER0 — elapsed-time and ECHO timestamps, /64 at 1 MHz.
// 1 tick = 64 us. Poll the overflow flag instead of interrupting every GPS
// character. The flag is serviced during reception and every main-loop pass.
// Motor PWM stays on Timer1; its counter is also READ for GPS bit timing.
// ============================================================
volatile uint32_t timer0_overflow_count = 0;

static inline __attribute__((always_inline)) void Timer0_service(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (TIFR & (1 << TOV0))
        {
            TIFR = (1 << TOV0);
            timer0_overflow_count++;
        }
    }
}

void Timer0_init()
{
    TCCR0 = (1 << CS01) | (1 << CS00); // /64; overflow every 16.384 ms
    TIMSK &= ~(1 << TOIE0);
    TIFR = (1 << TOV0);
}

uint32_t get_time_us()
{
    uint32_t ovf;
    uint8_t cnt;
    Timer0_service();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ovf = timer0_overflow_count;
        cnt = TCNT0;
        if ((TIFR & (1 << TOV0)) && cnt < 255) ++ovf;
    }
    return ((ovf << 8) | cnt) * 64UL;
}

// ============================================================
// TIMER2 — original 100.352 ms sensor period at 1 MHz.
// The short assembly ISR only makes the trigger pulse and increments a tick.
// All obstacle-state bookkeeping is deferred to the main loop. This keeps
// the periodic interruption short enough to share time with GPS reception.
// ============================================================
volatile uint8_t timer2_ticks = 0;

#ifdef SCOUTBOT_HOST_TEST
ISR(TIMER2_COMP_vect) { timer2_ticks++; }
#else
ISR(TIMER2_COMP_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24\n\t"
        "in r24, __SREG__\n\t"
        "push r24\n\t"
        "sbi %[port], %[pin]\n\t"
        "ldi r24, 3\n\t"
        "1: dec r24\n\t"
        "brne 1b\n\t"
        "cbi %[port], %[pin]\n\t"
        "lds r24, timer2_ticks\n\t"
        "inc r24\n\t"
        "sts timer2_ticks, r24\n\t"
        "pop r24\n\t"
        "out __SREG__, r24\n\t"
        "pop r24\n\t"
        "reti\n\t"
        : : [port] "I" (_SFR_IO_ADDR(PORTC)), [pin] "I" (TRIG_PIN)
        : "memory"
    );
}
#endif

void service_ticks(void)
{
    static uint8_t previous = 0;
    uint8_t current = timer2_ticks;
    uint8_t elapsed = current - previous;
    previous = current;
    if (!elapsed) return;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (elapsed >= obstacle_cooldown) obstacle_cooldown = 0;
        else obstacle_cooldown -= elapsed;
    }
    if (obstacle_state != OBSTACLE_STATE_IDLE)
    {
        uint16_t ticks = (uint16_t)obstacle_action_ticks + elapsed;
        obstacle_action_ticks = ticks > 255 ? 255 : ticks;
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
volatile uint16_t echo_start_ticks = 0;
volatile uint8_t echo_started = 0;

ISR(INT0_vect)
{
    // No division or function calls on the GPS timing path. Only the low
    // 16 timer bits are needed: they span 4.19 s, much longer than an echo.
    uint8_t high = *((volatile uint8_t *)&timer0_overflow_count);
    uint8_t low = TCNT0;
    if ((TIFR & (1 << TOV0)) && low < 255) ++high;
    uint16_t now = ((uint16_t)high << 8) | low;
    if (PIND & (1 << PD2))
    {
        echo_start_ticks = now;
        echo_started = 1;
    }
    else if (echo_started)
    {
        echo_started = 0;
        uint16_t width = now - echo_start_ticks;
        if (width >= 1 && width < ((OBSTACLE_THRESHOLD_CM * 58U + 63U) / 64U) &&
            obstacle_cooldown == 0 && obstacle_state == OBSTACLE_STATE_IDLE &&
            obstacle_event == 0 && !stop_requested)
        {
            PORTB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
            pending_drive = 0;
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
// GPS MODULE (NEO-6M) — SOFTWARE UART RECEIVER + NMEA PARSER
// ------------------------------------------------------------
// Wiring:
//   GPS VCC -> voltage specified for your breakout board; bare NEO-6M = 2.7-3.6V
//   GPS GND -> common GND
//   GPS TX  -> ATmega32 PC1   (read-only; NEO-6M default = 9600 baud)
//   GPS RX  -> not connected  (so the module is never reconfigured
//              default is 9600 baud, 8N1; a saved configuration can differ)
//
// The ATmega32's only hardware USART is already committed to the
// ESP32-CAM link, and PC1 is not an external/pin-change-interrupt
// capable pin on this chip, so GPS reception is done with a small
// polled software UART instead of a second hardware peripheral.
//
// This version keeps the existing INTERNAL 1 MHz clock. GPS samples are
// timed against the existing Timer1 PWM counter, READ ONLY (8 us/tick).
// Timer1's motor PWM registers and output frequency remain unchanged.
// Timer0 overflow and queued serial output are serviced between bit samples;
// the sensor and STOP interrupts remain enabled. GPS reads one sentence at
// a time so main-loop bookkeeping cannot insert gaps between every byte.
// ============================================================
#define GPS_PIN PC1
#define GPS_BIT_TICKS 13 // 13 * 8 us = 104 us at 9600 baud

#define NMEA_BUF_SIZE 90

char    nmea_buf[NMEA_BUF_SIZE];
uint8_t nmea_idx = 0;

// Last known-good fix. These are only overwritten once a $..GGA
// sentence reports a valid fix, so a brief loss of satellite lock
// does not erase the last real location.
float   gps_latitude  = 0.0f;
float   gps_longitude = 0.0f;
uint8_t gps_fix_valid = 0; // 1 = most recently accepted GGA had a live fix
uint32_t gps_fix_time_us = 0;

uint8_t GPS_fix_is_fresh()
{
	if (gps_fix_valid && (uint32_t)(get_time_us() - gps_fix_time_us) >= 5000000UL)
	{
		gps_fix_valid = 0;
	}
	return gps_fix_valid;
}

void GPS_init()
{
	DDRC  &= ~(1 << GPS_PIN); // PC1 as input
	PORTC &= ~(1 << GPS_PIN); // Do not pull the GPS output up to ATmega VCC.
	// The powered GPS TX drives idle HIGH; verify compatible logic levels.
}

// Attempts to receive a single byte from the GPS module.
// Returns 0-255 on success, or -1 if the line is currently idle
// (nothing to read right now — this call did NOT block).
static inline __attribute__((always_inline)) int16_t GPS_try_receive_byte()
{
    static uint8_t saw_idle = 0;
    uint8_t wait_start = TCNT0;
    // Wait briefly for a start edge, rather than repeatedly running slow
    // main-loop housekeeping during the beginning of a GPS character.
    while (PINC & (1 << GPS_PIN))
    {
        saw_idle = 1;
        if ((uint8_t)(TCNT0 - wait_start) >= 16) return -1; // ~1 ms
    }
    if (!saw_idle) return -2;
    saw_idle = 0;
    uint8_t start = TCNT1L;
    while ((uint8_t)(TCNT1L - start) < 6) {}
    if (PINC & (1 << GPS_PIN)) return -2;

    uint8_t value = 0;
    uint8_t deadline = 19;
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
        while ((uint8_t)(TCNT1L - start) < deadline) {}
        if ((uint8_t)(TCNT1L - start) >= (uint8_t)(deadline + 6)) return -2;
        value >>= 1;
        if (PINC & (1 << GPS_PIN)) value |= 0x80;
        deadline += GPS_BIT_TICKS;
        if (bit == 3)
        {
            // Do bounded background work just AFTER a sample. The next
            // sample retains its absolute deadline, so overhead cannot drift.
            Timer0_service();
            UART_tx_poll();
        }
    }
    while ((uint8_t)(TCNT1L - start) < deadline) {}
    if (!(PINC & (1 << GPS_PIN))) return -2;
    saw_idle = 1;
    return value;
}

// Writes an unsigned 32-bit integer as decimal ASCII starting at p,
// with no leading zeros, and returns a pointer just past the last
// digit written (so callers can keep appending after it).
static char *write_uint32(uint32_t value, char *p)
{
	char tmp[10];
	uint8_t i = 0;

	if (value == 0)
	{
		tmp[i++] = '0';
	}
	else
	{
		while (value > 0)
		{
			tmp[i++] = '0' + (uint8_t)(value % 10);
			value /= 10;
		}
	}

	while (i > 0)
	{
		*p++ = tmp[--i];
	}

	return p;
}

// Formats a coordinate (decimal degrees) as "[-]DDD.DDDDDD" into out,
// using integer formatting after float scaling. This avoids dtostrf(),
// which is an avr-libc/Arduino-only extension and is not guaranteed to
// be present (or linked in) by every AVR toolchain/C library.
void format_coord(float value, char *out)
{
	int32_t scaled = (int32_t)(value * 1000000.0f + (value >= 0.0f ? 0.5f : -0.5f));

	char *p = out;

	if (scaled < 0)
	{
		*p++ = '-';
		scaled = -scaled;
	}

	uint32_t int_part  = (uint32_t)(scaled / 1000000L);
	uint32_t frac_part = (uint32_t)(scaled % 1000000L);

	p = write_uint32(int_part, p);
	*p++ = '.';

	// Zero-pad the fractional part to exactly 6 digits.
	char frac_digits[6];
	for (int8_t i = 5; i >= 0; i--)
	{
		frac_digits[i] = '0' + (uint8_t)(frac_part % 10);
		frac_part /= 10;
	}
	for (uint8_t i = 0; i < 6; i++)
	{
		*p++ = frac_digits[i];
	}

	*p = '\0';
}

// Converts an NMEA "DDMM.MMMMM" / "DDDMM.MMMMM" field into decimal
// degrees. deg_digits is 2 for latitude, 3 for longitude.
float nmea_to_decimal(const char *raw, uint8_t deg_digits)
{
	char deg_part[4] = {0, 0, 0, 0};

	for (uint8_t i = 0; i < deg_digits && raw[i] != '\0'; i++)
	{
		deg_part[i] = raw[i];
	}

	float degrees = (float)atoi(deg_part);
	float minutes = atof(raw + deg_digits);

	return degrees + (minutes / 60.0f);
}

// Sends a live telemetry line to the ESP32-CAM every time a GGA
// sentence is parsed, whether or not it currently has a fix:
//   T,<fixQuality>,<satellites>,<lat>,<lon>,<fixFlag>\n
//
// This is what lets you actually SEE what the GPS module is doing:
//   - No "T," lines ever show up on the ESP32 serial monitor
//     -> no complete validated GGA reached the ESP32. Check wiring,
//        baud/clock, software UART timing and NMEA output configuration.
//   - "T,0,0,0.000000,0.000000,0" repeating
//     -> a checksum-valid GGA reports no fix. Test outdoors with
//        a clear view of the sky and allow several minutes for acquisition.
//   - "T,1,N,<real lat>,<real lon>,1" (N > 0)
//     -> fix acquired; obstacle events will now report real coordinates.
void GPS_send_telemetry_to_esp(uint8_t fix_quality, uint8_t satellites)
{
	char lat_str[16];
	char lon_str[16];
	char num_str[4];
	char *p;

	format_coord(gps_latitude, lat_str);
	format_coord(gps_longitude, lon_str);

	UART_send_string("T,");

	p = write_uint32(fix_quality, num_str);
	*p = '\0';
	UART_send_string(num_str);
	UART_send_string(",");

	p = write_uint32(satellites, num_str);
	*p = '\0';
	UART_send_string(num_str);
	UART_send_string(",");

	UART_send_string(lat_str);
	UART_send_string(",");
	UART_send_string(lon_str);
	UART_send_string(",");
	UART_send(GPS_fix_is_fresh() ? '1' : '0');
	UART_send_string("\n");
}

// Reject corrupted software-UART data before trusting coordinates.
static int8_t nmea_hex(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}

static uint8_t nmea_checksum_ok(const char *line)
{
	if (*line++ != '$') return 0;
	uint8_t checksum = 0;
	while (*line && *line != '*') checksum ^= (uint8_t)*line++;
	if (*line != '*' || strlen(line) != 3) return 0;
	int8_t hi = nmea_hex(line[1]);
	int8_t lo = nmea_hex(line[2]);
	return hi >= 0 && lo >= 0 && checksum == (uint8_t)((hi << 4) | lo);
}

static uint8_t nmea_coord_ok(const char *raw, uint8_t deg_digits)
{
	size_t len = strlen(raw);
	uint8_t whole = deg_digits + 2;
	if (len < whole) return 0;
	for (uint8_t i = 0; i < whole; ++i)
		if (raw[i] < '0' || raw[i] > '9') return 0;
	if (len > whole)
	{
		if (raw[whole] != '.' || len == whole + 1U) return 0;
		for (size_t i = whole + 1; i < len; ++i)
			if (raw[i] < '0' || raw[i] > '9') return 0;
	}
	float minutes = atof(raw + deg_digits);
	float degrees = nmea_to_decimal(raw, deg_digits);
	return minutes < 60.0f && degrees <= (deg_digits == 2 ? 90.0f : 180.0f);
}

// Parse complete GGA sentences, preserving empty fields.
void GPS_parse_sentence(char *line)
{
	if (strlen(line) < 10 || !nmea_checksum_ok(line)) return;
	if (strncmp(line + 3, "GGA,", 4) != 0) return;

	char *fields[15];
	uint8_t field_count = 0;
	char *p = line;
	fields[field_count++] = p;
	while (*p != '\0' && field_count < 15)
	{
		if (*p == ',')
		{
			*p = '\0';
			fields[field_count++] = p + 1;
		}
		p++;
	}
	if (field_count < 8) return;
	if (strlen(fields[6]) != 1 || fields[6][0] < '0' || fields[6][0] > '8') return;
	if (strlen(fields[7]) < 1 || strlen(fields[7]) > 2) return;
	for (p = fields[7]; *p; ++p)
		if (*p < '0' || *p > '9') return;

	uint8_t fix_quality = (uint8_t)atoi(fields[6]);
	uint8_t satellites = (uint8_t)atoi(fields[7]);
	gps_fix_valid = 0;
	// GGA quality 6 = estimated/dead reckoning, 7 = manual, 8 = simulation.
	if (fix_quality >= 1 && fix_quality <= 5 &&
		nmea_coord_ok(fields[2], 2) && nmea_coord_ok(fields[4], 3) &&
		(strlen(fields[3]) == 1 && (fields[3][0] == 'N' || fields[3][0] == 'S')) &&
		(strlen(fields[5]) == 1 && (fields[5][0] == 'E' || fields[5][0] == 'W')))
	{
		float lat = nmea_to_decimal(fields[2], 2);
		float lon = nmea_to_decimal(fields[4], 3);
		gps_latitude = fields[3][0] == 'S' ? -lat : lat;
		gps_longitude = fields[5][0] == 'W' ? -lon : lon;
		gps_fix_time_us = get_time_us();
		gps_fix_valid = 1;
	}
	// Publish AFTER updating position and validity, including loss of fix.
	GPS_send_telemetry_to_esp(fix_quality, satellites);
}

// Read at most one sentence per main pass (normally under 90 ms).
// Sensor braking and manual STOP stay interrupt-driven during this call.
void GPS_poll()
{
    for (uint8_t count = 0; count < NMEA_BUF_SIZE; ++count)
    {
        int16_t rx = GPS_try_receive_byte();
        if (rx < 0)
        {
            if (rx == -2) nmea_idx = 0;
            return;
        }
        char c = (char)rx;
        if (c == '$') { nmea_idx = 0; nmea_buf[nmea_idx++] = c; }
        else if (nmea_idx && c != '\r')
        {
            if (c == '\n')
            {
                nmea_buf[nmea_idx] = 0;
                GPS_parse_sentence(nmea_buf);
                nmea_idx = 0;
                return;
            }
            if (nmea_idx < NMEA_BUF_SIZE - 1) nmea_buf[nmea_idx++] = c;
            else { nmea_idx = 0; return; }
        }
    }
}

// Sends the most recent GPS fix to the ESP32-CAM as a single line:
//   G,<lat>,<lon>,<fixFlag>\n
// fixFlag is '1' only if the sentence that produced these coordinates
// had a live fix; '0' means these are the last known coordinates from
// an earlier fix (still useful, just possibly stale).
void GPS_send_fix_to_esp()
{
	char lat_str[16];
	char lon_str[16];

	format_coord(gps_latitude, lat_str);
	format_coord(gps_longitude, lon_str);

	UART_send_string("G,");
	UART_send_string(lat_str);
	UART_send_string(",");
	UART_send_string(lon_str);
	UART_send_string(",");
	UART_send(GPS_fix_is_fresh() ? '1' : '0');
	UART_send_string("\n");
}

// ============================================================
// AUTOMATIC OBSTACLE HANDLER
// ============================================================
// Kept in the normal main loop so interrupts remain available.
// Sequence:
//   1) Immediate brake already applied by INT0 ISR.
//   2) Notify ESP32 to capture a photo, and send it the last known
//      GPS fix so it can tag the obstacle with a location.
//   3) Wait ~2 s for the photo/display pipeline.
//   4) Reverse for ~2 s at 40% speed.
//   5) Brake, then stop and restore the user's previous speed preset.
// ============================================================
void start_obstacle_response()
{
    uint8_t started = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (obstacle_event && !stop_requested)
        {
            obstacle_event = 0;
            pending_drive = 0;
            obstacle_state = OBSTACLE_STATE_WAIT;
            obstacle_action_ticks = 0;
            brakeMotors();
            started = 1;
        }
    }
    if (!started) return;
    saved_speed_before_obstacle = current_speed;
    // Coordinates FIRST: the ESP32 can associate the fix with this photo.
    GPS_send_fix_to_esp();
    UART_send_string("O\n");
}

void service_obstacle_response()
{
    if (stop_requested) return;
    if (obstacle_state == OBSTACLE_STATE_WAIT && obstacle_action_ticks >= OBSTACLE_WAIT_TICKS)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            if (!stop_requested)
            {
                obstacle_action_ticks = 0;
                obstacle_state = OBSTACLE_STATE_REVERSE;
                set_speed(OBSTACLE_REVERSE_SPEED);
            }
        }
        backward();
    }
    else if (obstacle_state == OBSTACLE_STATE_REVERSE && !pending_drive &&
             obstacle_action_ticks >= OBSTACLE_REVERSE_TICKS)
    {
        uint32_t deadline = get_time_us() + 100000UL;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            if (!stop_requested)
            {
                brakeMotors();
                brake_until_us = deadline;
                obstacle_state = OBSTACLE_STATE_BRAKE;
            }
        }
    }
    else if (obstacle_state == OBSTACLE_STATE_BRAKE &&
             (int32_t)(get_time_us() - brake_until_us) >= 0)
    {
        stopMotors();
        set_speed(saved_speed_before_obstacle);
        obstacle_state = OBSTACLE_STATE_IDLE;
    }
}

void service_stop()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (stop_requested)
        {
            stopMotors();
            pending_drive = 0;
            if (obstacle_state != OBSTACLE_STATE_IDLE) set_speed(saved_speed_before_obstacle);
            obstacle_state = OBSTACLE_STATE_IDLE;
            obstacle_event = 0;
            obstacle_action_ticks = 0;
            stop_requested = 0;
        }
    }
}

// ============================================================
// MAIN — handles incoming commands, GPS reception, and automatic
// obstacle response.
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
	GPS_init();

	sei(); // enable global interrupts

	while (1)
	{
        Timer0_service();
        UART_tx_poll();
        GPS_poll();
        Timer0_service();
        service_ticks();

		if (obstacle_event && obstacle_state == OBSTACLE_STATE_IDLE)
		{
			start_obstacle_response();
		}

        service_stop();
		service_obstacle_response();
        service_drive();

		if (UART_available())
		{
			char cmd = UART_receive();

			// Manual STOP is always honored. Other movement commands are
			// ignored while the automatic obstacle response is active so a
			// browser click cannot immediately defeat the safety maneuver.
			if (cmd == 'S')
			{
                stop_requested = 1;
                service_stop();
			}
			else if (obstacle_state == OBSTACLE_STATE_IDLE && !obstacle_event && !stop_requested)
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