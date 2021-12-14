/*
 * Copyright (C) 2021 Fern H. (aka Pavel Neshumov), Liberty-X Flight controller
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * IT IS STRICTLY PROHIBITED TO USE THE PROJECT (OR PARTS OF THE PROJECT / CODE)
 * FOR MILITARY PURPOSES. ALSO, IT IS STRICTLY PROHIBITED TO USE THE PROJECT (OR PARTS OF THE PROJECT / CODE)
 * FOR ANY PURPOSE THAT MAY LEAD TO INJURY, HUMAN, ANIMAL OR ENVIRONMENTAL DAMAGE.
 * ALSO, IT IS PROHIBITED TO USE THE PROJECT (OR PARTS OF THE PROJECT / CODE) FOR ANY PURPOSE THAT
 * VIOLATES INTERNATIONAL HUMAN RIGHTS OR HUMAN FREEDOM.
 * BY USING THE PROJECT (OR PART OF THE PROJECT / CODE) YOU AGREE TO ALL OF THE ABOVE RULES.
 */

#ifdef TELEMETRY

/// <summary>
/// Sends the telemetry data to the ground station
/// </summary>
void telemetry(void) {
	// Increment the telemetry_loop_counter variable every cycle
	telemetry_loop_counter++;

	if (telemetry_loop_counter == 1) {
		// Reset check byte after previos transmittion and suffix
		telemetry_check_byte = 0;

		// Send the error as a byte
		telemetry_send_byte = error;
	}

	// Send the flight mode as a byte
	else if (telemetry_loop_counter == 2)
		telemetry_send_byte = flight_mode;

	// Send the battery voltage as a byte.
	else if (telemetry_loop_counter == 3)
		telemetry_send_byte = (uint8_t)(battery_voltage * 10.0);

	else if (telemetry_loop_counter == 4) {
		// Store the temperature as it can change during the next loop
		telemetry_buffer_bytes = temperature;

		// Send the first 8 bytes of the temperature variable
		telemetry_send_byte = telemetry_buffer_bytes >> 8;
	}

	// Send the last 8 bytes of the temperature variable
	else if (telemetry_loop_counter == 5)
		telemetry_send_byte = telemetry_buffer_bytes;

	// Send the roll angle as a byte. Add 100 to prevent negative numbers
	else if (telemetry_loop_counter == 6)
		telemetry_send_byte = angle_roll + 100;

	// Send the pitch angle as a byte. Add 100 to prevent negative numbers
	else if (telemetry_loop_counter == 7)
		telemetry_send_byte = angle_pitch + 100;

	// Send the start status as a byte
	else if (telemetry_loop_counter == 8)
		telemetry_send_byte = start;

	else if (telemetry_loop_counter == 9) {
		if (start == 2) {
			// Send the altitude only when the quadcopter is flying
			// Calculate the altitude and add 1000 to prevent negative numbers
			telemetry_buffer_bytes = 1000 + ((ground_pressure - actual_pressure) * 0.0842f);
		}
		else {
			// Send and altitude of 0 meters if the quadcopter isn't flying
			telemetry_buffer_bytes = 1000;
		}
		// Send the first 8 bytes of the altitude variable
		telemetry_send_byte = telemetry_buffer_bytes >> 8;
	}

	//Send the last 8 bytes of the altitude variable
	else if (telemetry_loop_counter == 10)
		telemetry_send_byte = telemetry_buffer_bytes;

	else if (telemetry_loop_counter == 11) {
		// Store the take-off throttle as it can change during the next loop
		telemetry_buffer_bytes = 1500 + takeoff_throttle;

		// Send the first 8 bytes of the take-off throttle variable
		telemetry_send_byte = telemetry_buffer_bytes >> 8;
	}

	// Send the last 8 bytes of the take-off throttle variable
	else if (telemetry_loop_counter == 12)
		telemetry_send_byte = telemetry_buffer_bytes;

	// Send the takeoff_detected variable as a byte
	else if (telemetry_loop_counter == 13)
		telemetry_send_byte = takeoff_detected;

	else if (telemetry_loop_counter == 14) {
		// Store the compass heading as it can change during the next loop
		telemetry_buffer_bytes = angle_yaw;

		// Send the first 8 bytes of the compass heading variable
		telemetry_send_byte = telemetry_buffer_bytes >> 8;
	}

	// Send the last 8 bytes of the compass heading variable
	else if (telemetry_loop_counter == 15)
		telemetry_send_byte = telemetry_buffer_bytes;

	// Send the heading_lock variable as a byte
	else if (telemetry_loop_counter == 16)
		telemetry_send_byte = heading_lock_enabled;

	else if (telemetry_loop_counter == 17) {
		// Store the latitude position as it can change during the next loop
		telemetry_buffer_bytes = l_lat_gps;

		// Send the first 8 bytes of the latitude position variable
		telemetry_send_byte = telemetry_buffer_bytes >> 24;
	}

	// Send the next bytes of the latitude position variable
	else if (telemetry_loop_counter == 18)telemetry_send_byte = telemetry_buffer_bytes >> 16;
	else if (telemetry_loop_counter == 19)telemetry_send_byte = telemetry_buffer_bytes >> 8;
	else if (telemetry_loop_counter == 20)telemetry_send_byte = telemetry_buffer_bytes;

	else if (telemetry_loop_counter == 21) {
		// Store the longitude position as it can change during the next loop
		telemetry_buffer_bytes = l_lon_gps;

		// Send the first 8 bytes of the longitude position variable
		telemetry_send_byte = telemetry_buffer_bytes >> 24;
	}

	// Send the next 8 bytes of the longitude position variable
	else if (telemetry_loop_counter == 22)telemetry_send_byte = telemetry_buffer_bytes >> 16;
	else if (telemetry_loop_counter == 23)telemetry_send_byte = telemetry_buffer_bytes >> 8;
	else if (telemetry_loop_counter == 24)telemetry_send_byte = telemetry_buffer_bytes;

	// Send the number_used_sats variable as a byte
	else if (telemetry_loop_counter == 25)
		telemetry_send_byte = number_used_sats;

	else if (telemetry_loop_counter == 26) {
		// Store ground speed as it can change during the next loop
		telemetry_buffer_bytes = ground_speed;

		// Send first 8 bytes of ground_speed variable
		telemetry_send_byte = telemetry_buffer_bytes >> 8;
	}

	// Send last 8 bytes of the ground_speed variable
	else if (telemetry_loop_counter == 27)
		telemetry_send_byte = telemetry_buffer_bytes;

#ifdef LIBERTY_LINK
	// Send waypoints flight step if Liberty-Link is enabled
	else if (telemetry_loop_counter == 28) {
		if (!auto_landing_step)
			telemetry_send_byte = link_waypoint_step;
		else
			telemetry_send_byte = (uint8_t)128 + auto_landing_step;
	}

	// Send waypoint index if Liberty-Link is enabled
	else if (telemetry_loop_counter == 29)
		telemetry_send_byte = waypoints_index;
#else
	// Send nothing if Liberty-Link is disabled
	else if (telemetry_loop_counter == 28) {
		if (!auto_landing_step)
			telemetry_send_byte = 0;
		else
			telemetry_send_byte = (uint8_t)128 + auto_landing_step;
	}
	else if (telemetry_loop_counter == 29)
		telemetry_send_byte = 0;
#endif

#ifdef SONARUS
	// Send sonarus distance
	else if (telemetry_loop_counter == 30)
		telemetry_send_byte = sonarus_bottom_compressed;
#else
	// Send nothing if Sonarus is disabled
	else if (telemetry_loop_counter == 30)
		telemetry_send_byte = 0;
#endif // SONARUS


#ifdef LUX_METER
	// Send ambient illumination
	else if (telemetry_loop_counter == 31) {
		telemetry_send_byte = lux_sqrt_data + 1;
	}
#else
	// Send nothing if lux meter is disabled
	else if (telemetry_loop_counter == 31)
		telemetry_send_byte = 0;
#endif

	// Send the check-byte
	else if (telemetry_loop_counter == 32)telemetry_send_byte = telemetry_check_byte;

	// Send the first suffix
	else if (telemetry_loop_counter == 33)telemetry_send_byte = TELEMETRY_SUFFIX_1;

	// Send the second suffix
	else if (telemetry_loop_counter == 34)telemetry_send_byte = TELEMETRY_SUFFIX_2;

	if (telemetry_loop_counter > 0 && telemetry_loop_counter <= 34) {
		// XOR every send_byte
		telemetry_check_byte ^= telemetry_send_byte;

		// Push data to the serial port
		TELEMETRY_SERIAL.write(telemetry_send_byte);
	}

	// Reset the telemetry_loop_counter variable after 125 loops. This way the telemetry data is send every 125 * 4ms = 500ms
	if (telemetry_loop_counter >= 125)
		telemetry_loop_counter = 0;
}

#endif

