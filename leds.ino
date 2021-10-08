/*
 * Copyright (C) 2021 Frey Hertz (Pavel Neshumov), Liberty-X Flight controller
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/// <summary>
/// Initializes onboard and the WS2812 LEDs
/// </summary>
void leds_setup(void) {
	// Turn on builtin LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, 1);

	// Setup SPI for WS2812
	SPI.setModule(2);
	SPI.setClockDivider(SPI_CLOCK_DIV16);

	// Initilize WS2812
	ws_leds.begin();
	leds_idle_signal();
}

/// <summary>
/// Shows current drone state with the LEDs. This void exetues in main loop
/// </summary>
void leds_handler(void) {
	leds_flight_mode_signal();
	if (error > 0) {
		// Error exists
		leds_error_signal();
	}
	else if (start > 0) {
		// Static and blick in flight and no error
		leds_in_flight_signal();
	}
	else {
		// Show IDLE rainbow sweep on land and no error
		leds_idle_signal();
	}

	// Update WS2812 LEDs
	ws_leds.show();
}

/// <summary>
/// Shows static green-red and blinking whute signals
/// </summary>
void leds_in_flight_signal(void) {
	// Increment loop counter
	leds_loop_counter++;

	if (leds_tick_counter && leds_loop_counter >= 10) {
		// Reset counters
		leds_tick_counter = 0;
		leds_loop_counter = 0;
	}
	if (start > 1 && takeoff_detected && !leds_tick_counter && leds_loop_counter >= LEDS_BLINK_CYCLES) {
		// If in flight and takeoff detected
		// Reset counters
		leds_tick_counter = 1;
		leds_loop_counter = 0;
	}

	if (leds_tick_counter) {
		// Show blink after static colors
		leds_blink_signal();
	}
	else {
		// Show static colors after blink of 10 ms
		leds_static_signal();
	}
}

/// <summary>
/// Shows green-white signals and Liberty-Link operating signals
/// </summary>
void leds_static_signal(void) {
	// Static colors
#ifdef LIBERTY_LINK
	if (link_lost_counter < LINK_LOST_CYCLES && link_allowed) {
		// Show link colors if the Liberty-Link is on and not in IDLE mode
		ws_leds.setPixelColor(1, COLOR_FRONT_LINK);
		ws_leds.setPixelColor(2, COLOR_REAR_LINK);
	}
	else {
		// Show default colors if the Liberty-Link is off
		ws_leds.setPixelColor(1, COLOR_FRONT);
		ws_leds.setPixelColor(2, COLOR_REAR);
	}
#else
	// Show default colors if the Liberty-Link is disabled
	ws_leds.setPixelColor(1, COLOR_FRONT);
	ws_leds.setPixelColor(2, COLOR_REAR);
#endif
}

/// <summary>
/// Blinks with COLOR_BLINK
/// </summary>
void leds_blink_signal(void) {
	// Blink with COLOR_BLINK
	ws_leds.setPixelColor(1, COLOR_BLINK);
	ws_leds.setPixelColor(2, COLOR_BLINK);
}

/// <summary>
/// Sweeps through all colors (IDLE rainbow)
/// </summary>
void leds_idle_signal(void) {
	// Increment loop counter
	leds_loop_counter++;

	if (leds_loop_counter >= LEDS_IDLE_CYCLES) {
		// Reset loop counter
		leds_loop_counter = 0;

		// Rainbow sweep
		if (leds_tick_counter < 85)
		{
			ws_leds.setPixelColor(1, leds_tick_counter * 3, 255 - leds_tick_counter * 3, 0);
			ws_leds.setPixelColor(2, leds_tick_counter * 3, 255 - leds_tick_counter * 3, 0);
		}
		else if (leds_tick_counter < 170)
		{
			ws_leds.setPixelColor(1, 255 - (leds_tick_counter - 85) * 3, 0, (leds_tick_counter - 85) * 3);
			ws_leds.setPixelColor(2, 255 - (leds_tick_counter - 85) * 3, 0, (leds_tick_counter - 85) * 3);
		}
		else
		{
			ws_leds.setPixelColor(1, 0, (leds_tick_counter - 170) * 3, 255 - (leds_tick_counter - 170) * 3);
			ws_leds.setPixelColor(2, 0, (leds_tick_counter - 170) * 3, 255 - (leds_tick_counter - 170) * 3);
		}

		// Sweep all colors
		if (leds_tick_counter < 255)
			leds_tick_counter++;
		else
			leds_tick_counter = 0;
	}
}

/// <summary>
/// Blinks with COLOR_CALIBRATION. Useful to indicate the calibration process
/// </summary>
void leds_calibration_signal(void) {
	// Change flag
	leds_tick_counter = !leds_tick_counter;

	// Switch colors
	if (leds_tick_counter) {
		ws_leds.setPixelColor(0, COLOR_CALIBRATION);
		ws_leds.setPixelColor(1, COLOR_CALIBRATION);
		ws_leds.setPixelColor(2, COLOR_CALIBRATION);
	}
	else {
		ws_leds.clear();
	}
	ws_leds.show();
}

/// <summary>
/// Blinks with red onboard LED and COLOR_ERROR as many times as the error is
/// </summary>
void leds_error_signal(void) {
	// Increment loop counter
	leds_error_loop_counter++;

	if (leds_error_loop_counter >= LEDS_ERROR_CYCLES) {
		// Reset loop counter
		leds_error_loop_counter = 0;

		// Reset leds_error_counter after +3 cycles (for delay)
		if (error > 0 && leds_error_counter > error + 3) 
			leds_error_counter = 0;
		if (leds_error_counter < error && !leds_red_state && error > 0) {
			// Turn LEDs on if the error flash sequence isn't finished and the red LED is off
			leds_onboard(1, leds_green_state);
			ws_leds.setPixelColor(1, COLOR_ERROR);
			ws_leds.setPixelColor(2, COLOR_ERROR);
		} else {
			// Turn LEDs off if the error flash sequence isn't finished and the red LED is on
			leds_onboard(0, leds_green_state);
			ws_leds.setPixelColor(1, 0);
			ws_leds.setPixelColor(2, 0);

			// Increment counter
			leds_error_counter++;
		}

		// Update WS2812 LEDs
		ws_leds.show();
	}
}

/// <summary>
/// Blinks with green onboard LED as many times as the flight_mode is
/// </summary>
void leds_flight_mode_signal(void) {
	// Increment loop counter
	leds_onboard_loop_counter++;

	if (leds_onboard_loop_counter >= LEDS_FLIGHT_MODE_CYCLES) {
		// Reset loop counter
		leds_onboard_loop_counter = 0;

		// Reset leds_flight_mode_counter after +3 cycles (for delay)
		if (flight_mode > 0 && leds_flight_mode_counter > flight_mode + 3) 
			leds_flight_mode_counter = 0;
		if (leds_flight_mode_counter < flight_mode && !leds_green_state && flight_mode > 0) {
			// Turn green LED on if the flight mode flash sequence isn't finished and the red LED is off
			leds_onboard(leds_red_state, 1);
		}
		else {
			// Turn green LED off if the flight mode flash sequence isn't finished and the green LED is on
			leds_onboard(leds_red_state, 0);

			// Increment counter
			leds_flight_mode_counter++;
		}
	}
}

/// <summary>
/// Changes state of the onboard LEDs
/// </summary>
/// <param name="red"> New red LED value </param>
/// <param name="green"> New green LED value </param>
void leds_onboard(boolean red, boolean green) {
	// Update onboard led
	if (red && green)
		ws_leds.setPixelColor(0, 255, 255, 0);
	else if (red)
		ws_leds.setPixelColor(0, 255, 0, 0);
	else if (green)
		ws_leds.setPixelColor(0, 0, 255, 0);
	else
		ws_leds.setPixelColor(0, 0, 0, 0);
	ws_leds.show();

	// Store last values
	leds_red_state = red;
	leds_green_state = green;
}