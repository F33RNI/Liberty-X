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

/// <summary>
/// Initializes the gps baud rate
/// </summary>
void gps_setup(void) {
	// Open serial port
	GPS_SERIAL.begin(GPS_BAUD_RATE);
	delay(200);
}

/// <summary>
/// Reads GPS data from the serial port
/// </summary>
void gps_read(void) {
	// Signal lost watchdog counter
	if (gps_lost_counter < UINT8_MAX)
		gps_lost_counter++;

	// Count number of cycles between each GPS measurment
	if (gps_lost_counter < GPS_LOST_CYCLES)
		gps_cycles_counter++;

	while (GPS_SERIAL.available()) {
		// Read current byte
		gps_buffer[gps_buffer_position] = GPS_SERIAL.read();

		if (gps_byte_previous == GPS_SUFFIX_1 && gps_buffer[gps_buffer_position] == GPS_SUFFIX_2) {
			// If data suffix appears
			// Reset buffer position
			gps_buffer_position = 0;

			// Reset check sum
			gps_check_byte = 0;

			// Calculate check sum
			for (gps_temp_byte = 0; gps_temp_byte <= 16; gps_temp_byte++)
				gps_check_byte ^= gps_buffer[gps_temp_byte];

			if (gps_check_byte == gps_buffer[17]) {
				// If the check sums are equal
				// Reset watchdog
				gps_lost_counter = 0;

				// GPS position
				l_lat_gps = (int32_t)gps_buffer[3] | (int32_t)gps_buffer[2] << 8 | (int32_t)gps_buffer[1] << 16 | (int32_t)gps_buffer[0] << 24;
				l_lon_gps = (int32_t)gps_buffer[7] | (int32_t)gps_buffer[6] << 8 | (int32_t)gps_buffer[5] << 16 | (int32_t)gps_buffer[4] << 24;

				// Number of satellites
				number_used_sats = gps_buffer[9];

				// HDOP (multiplied by 10)
				hdop = gps_buffer[10];

				// Altitude (multiplied by 10)
				altitude = (int16_t)gps_buffer[12] | (int16_t)gps_buffer[11] << 8;

				// Ground heading (multiplied by 100)
				ground_heading = (uint16_t)gps_buffer[14] | (uint16_t)gps_buffer[13] << 8;

				// Ground speed (multiplied by 10)
				ground_speed = (uint16_t)gps_buffer[16] | (uint16_t)gps_buffer[15] << 8;

				// Set new data flag
				new_gps_data_available = 1;

				// Initilize prevoius varianbles if this is the first time the GPS code is used
				if ((lat_gps_previous == 0 && lon_gps_previous == 0) || gps_cycles_counter == 0) {
					lat_gps_previous = l_lat_gps;
					lon_gps_previous = l_lon_gps;
					lat_gps_loop_add = 0;
					lon_gps_loop_add = 0;
				}

				// Calculate loop_add for GPS predistions
				else {
					lat_gps_loop_add = (float)(l_lat_gps - lat_gps_previous) / (float)gps_cycles_counter;
					lon_gps_loop_add = (float)(l_lon_gps - lon_gps_previous) / (float)gps_cycles_counter;
					lat_gps_loop_add *= (float)GPS_PREDICT_AFTER_CYCLES;
					lon_gps_loop_add *= (float)GPS_PREDICT_AFTER_CYCLES;
				}

				// Start GPS predistions
				gps_add_counter = GPS_PREDICT_AFTER_CYCLES;

				// Reset cycle counter
				gps_cycles_counter = 0;

				// Remember new latitude and longitude values
				lat_gps_previous = l_lat_gps;
				lon_gps_previous = l_lon_gps;
			}
			else
				gps_lost_counter = UINT8_MAX;
		}
		else {
			// Store data bytes
			gps_byte_previous = gps_buffer[gps_buffer_position];
			gps_buffer_position++;

			// Reset buffer on overflow
			if (gps_buffer_position > 19)
				gps_buffer_position = 0;
		}
	}

	// GPS prediction every GPS_PREDICT_AFTER_CYCLES program loops GPS_PREDICT_AFTER_CYCLES x 4ms
	if (gps_add_counter > 0)
		gps_add_counter--;
	else if (gps_lost_counter < GPS_LOST_CYCLES) {
		// Reset loop counter
		gps_add_counter = GPS_PREDICT_AFTER_CYCLES;

		// Set new_gps_data_available flag
		new_gps_data_available = 1;

		// Add the simulated part to a buffer float variables because the l_lat_gps and l_lon_gps can only hold integers
		lat_gps_add += lat_gps_loop_add;
		lon_gps_add += lon_gps_loop_add;

		// If the absolute value of lat_gps_add is larger then 1
		if (abs(lat_gps_add) >= 1) {

			// Increment the lat_gps_add value with the lat_gps_add value as an integer
			l_lat_gps += (int)lat_gps_add;

			// Subtract the lat_gps_add value as an integer so the decimal value remains
			lat_gps_add -= (int)lat_gps_add;
		}
		
		// If the absolute value of lon_gps_add is larger then 1
		if (abs(lon_gps_add) >= 1) {

			// Increment the lon_gps_add value with the lat_gps_add value as an integer
			l_lon_gps += (int)lon_gps_add;

			// Subtract the lon_gps_add value as an integer so the decimal value remains
			lon_gps_add -= (int)lon_gps_add;
		}
	}

	if (gps_lost_counter > GPS_LOST_CYCLES) {
		// When there is no GPS information available
		// Turn off builtin LED
		set_buildin_led(0);

		// Reset some variables
		l_lat_gps = 0;
		l_lon_gps = 0;
		number_used_sats = 0;
		new_gps_data_available = 0;
		lat_gps_previous = 0;
		lon_gps_previous = 0;
	}
}

/// <summary>
/// Handles new GPS data, executes PID controller and changes setpoints using the sticks
/// </summary>
void gps_handler(void) {
	if (new_gps_data_available) {
		// If there is a new set of GPS data available
		// Change the LED on the STM32 to indicate GPS reception
		if (number_used_sats < 8)
			set_buildin_led(!buildin_led_state);
		else 
			set_buildin_led(1);

		if (flight_mode >= 3 && !gps_setpoint_set) {
			// Remember the current location as setpoint if the flight mode is 3 (GPS hold) and no setpoints are set
			gps_setpoint_set = 1;
			l_lat_setpoint = l_lat_gps;
			l_lon_setpoint = l_lon_gps;
			pid_gps_reset();
		}

		// If the GPS hold mode and the setpoints are stored
		if (flight_mode >= 3 && gps_setpoint_set) {
			if (flight_mode == 3 && takeoff_detected) {
				// GPS stick move adjustments
				l_lat_gps_float_adjust -= 0.0015 * (((channel_2 - 1500) * cos(gps_man_adjust_heading * DEG_TO_RAD))
					+ ((channel_1 - 1500) * cos((gps_man_adjust_heading - 90) * DEG_TO_RAD)));

				l_lon_gps_float_adjust += (0.0015 * (((channel_1 - 1500) * cos(gps_man_adjust_heading * DEG_TO_RAD))
					+ ((channel_2 - 1500) * cos((gps_man_adjust_heading + 90) * DEG_TO_RAD)))) / cos(((float)l_lat_gps / 1000000.0) * DEG_TO_RAD);
			}

			// Adjust l_lat_setpoint with float correction
			if (l_lat_gps_float_adjust > 1) {
				l_lat_setpoint++;
				l_lat_gps_float_adjust--;
			}
			if (l_lat_gps_float_adjust < -1) {
				l_lat_setpoint--;
				l_lat_gps_float_adjust++;
			}

			// Adjust l_lon_setpoint with float correction
			if (l_lon_gps_float_adjust > 1) {
				l_lon_setpoint++;
				l_lon_gps_float_adjust--;
			}
			if (l_lon_gps_float_adjust < -1) {
				l_lon_setpoint--;
				l_lon_gps_float_adjust++;
			}
		}
	}

	if (gps_lost_counter > GPS_LOST_CYCLES) {
		// If the watchdog timer is exceeded the GPS signal is missing
		// Reset number of satellites
		number_used_sats = 0;

		if (flight_mode >= 3 && start > 0) {
			// Set the flight mode to 2 (Altitude hold) if flight mode is set to 3 (GPS hold)
			flight_mode = 2;
			// Output an error
			error = 4;
		}
	}

	if (flight_mode < 3 && gps_setpoint_set > 0) {
		// Reset variables to disable the correction if the GPS hold mode is disabled and the setpoints are set
		gps_setpoint_set = 0;
		l_lat_setpoint = 0;
		l_lon_setpoint = 0;

		pid_gps_reset();
	}
}