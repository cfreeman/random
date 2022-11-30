/*
 * Copyright (c) Clinton Freeman 2022
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>


/**************************************************************************************************
 * CONFIGURATION
 *************************************************************************************************/
#define BLINK_DURATION 0.2  // The duration of the LED blink in seconds.
#define LENGTH 5.0          // The length of window in seconds in which the LED blink once.
/**************************************************************************************************
 **************************************************************************************************
 *************************************************************************************************/


// Variables for accumulating bytes in the background for the true random generator.
byte sample = 0;
boolean sample_waiting = false;
byte current_bit = 0;
byte result = 0;
byte rotl(const byte value, int shift);

typedef struct State_struct (*ModeFn)(struct State_struct current_state,
									  unsigned long current_time);

typedef struct State_struct {
	unsigned long blink_start;      // The time the blink will be in milliseconds.
	unsigned long blink_duration;   // The duration of the blink in milliseconds.

	unsigned long started_at;       // The time this mode started at in milliseconds
	ModeFn update;                  // The current function used to update the state.
} State;

State StartMode(State current_state, unsigned long current_time);

State SetupMode(State current_state,
				unsigned long current_time) {
	Serial.println("SetupMode");

	// Add the hardware sample to the random number.
	if (sample_waiting) {
		sample_waiting = false;
		result = rotl(result, 1); // Spread randomness around
		result ^= sample;         // XOR preserves randomness
		current_bit++;

		// We have acculmulated enough data to build a random number.
		if (current_bit > 7) {
			current_bit = 0;

			// Determine blink details and start the random window.
			State new_state;
			new_state.blink_start = long((float(result) / 255.0) * LENGTH * 1000.0);
			new_state.blink_duration = current_state.blink_duration;
			new_state.started_at = millis();
			new_state.update = &StartMode;
			return new_state;
		}
	}

	return current_state;
}

State StartMode(State current_state,
				unsigned long current_time) {
	Serial.print(current_time - current_state.started_at);
	Serial.print(":");
	Serial.print(current_state.blink_start);
	Serial.println(" - StartMode");
	return current_state;
}


byte rotl(const byte value, int shift) {
	if ((shift &= sizeof(value)*8 - 1) == 0) {
		return value;
	}
	return (value << shift) | (value >> (sizeof(value)*8 - shift));
}

// Setup of the watchdog timer.
void wdtSetup() {
	cli();
	MCUSR = 0;

	/* Start timed sequence */
	WDTCSR |= _BV(WDCE) | _BV(WDE);

	/* Put WDT into interrupt mode */
	/* Set shortest prescaler(time-out) value = 2048 cycles (~16 ms) */
	WDTCSR = _BV(WDIE);

	sei();
}

State state;

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);
	wdtSetup();

	// Run the watchdog timer a bit before starting.
	delay(1000);

	// Initialise the state.
	state.blink_start = 0;
	state.blink_duration = BLINK_DURATION * 1000;
	state.started_at = millis();
	state.update = &SetupMode;
}

void render(State s, unsigned long t) {
	if (s.update != &StartMode) {
		return;
	}

	if (t > (s.started_at + s.blink_start) &&
		t < (s.started_at + s.blink_start + s.blink_duration)) {
		digitalWrite(LED_BUILTIN, HIGH);
	} else if (t > (s.started_at + s.blink_start + s.blink_duration)) {
		digitalWrite(LED_BUILTIN, LOW);
	}
}

void loop() {
	unsigned long t = millis();
	state = state.update(state, t);
	render(state, t);
}

// Watchdog Timer Interrupt Service Routine
ISR(WDT_vect) {
	sample = TCNT1L; // Ignore higher bits
	sample_waiting = true;
}