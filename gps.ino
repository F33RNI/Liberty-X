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
/// Initializes the compass on 57.6kbps baud rate
/// </summary>
void gps_setup(void) {
	// Open serial port at default baud rate
	GPS_SERIAL.begin(9600);
	delay(250);

	// Disable GPGSV messages by using the ublox protocol
	GPS_SERIAL.write(GPS_DISABLE_GPGSV, 11);
	delay(350);

	// Set the refresh rate to 5Hz by using the ublox protocol
	GPS_SERIAL.write(GPS_SET_TO_5HZ, 14);
	delay(350);

	// Set the baud rate to 57.6kbps by using the ublox protocol
	GPS_SERIAL.write(GPS_SET_TO_57KBPS, 28);
	delay(200);

	// Open serial port at 57.6kbps
	GPS_SERIAL.begin(57600);
	delay(200);
}

/// <summary>
/// Reads raw UBLOX data from the serial port
/// </summary>
void gps_read(void) {
	// Signal lost watchdog counter
	if (gps_lost_counter < UINT16_MAX)
		gps_lost_counter++;

	// GPS simulation counter
	if (gps_add_counter >= 0)
		gps_add_counter--;

	// Read data from the GPS module
	while (GPS_SERIAL.available() && new_line_found == 0) {
		// Stay in this loop as long as there is serial information from the GPS available
		read_serial_byte = GPS_SERIAL.read();
		if (read_serial_byte == '$') {
			// Clear the old data from the incoming buffer array if the new byte equals a $ character
			for (message_counter = 0; message_counter <= 99; message_counter++) {
				incoming_message[message_counter] = '-';
			}
			// Reset the message_counter variable because we want to start writing at the begin of the array
			message_counter = 0;
		}
		// If the received byte does not equal a $ character, increase the message_counter variable
		else if (message_counter <= 99)
			message_counter++;

		// Write the new received byte to the new position in the incoming_message array
		incoming_message[message_counter] = read_serial_byte;

		// Every NMEA line ends with a '*'. If this character is detected the new_line_found variable is set to 1
		if (read_serial_byte == '*')
			new_line_found = 1;
	}
}

/// <summary>
/// Parses GPS UBLOX protocol, executes PID controller and changes setpoints using the sticks
/// </summary>
void gps_handler(void) {
	// If the software has detected a new NMEA line it will check if it's a valid line that can be used
	if (new_line_found == 1) {
		// Reset the new_line_found variable for the next line
		new_line_found = 0;

		if (incoming_message[4] == 'L' && incoming_message[5] == 'L' && incoming_message[7] == ',') {
			// When there is no GPS fix or latitude/longitude information available
			// Turn off builtin LED
			digitalWrite(LED_BUILTIN, 1);

			// Set some variables to 0 if no valid information is found by the GPS module. This is needed for the GPS loss when flying
			l_lat_gps = 0;
			l_lon_gps = 0;
			lat_gps_previous = 0;
			lon_gps_previous = 0;
			number_used_sats = 0;
			gps_lost_counter = UINT16_MAX;
			
		}
		// If the line starts with GA and if there is a GPS fix we can scan the line for the latitude, longitude and number of satellites
		if (incoming_message[4] == 'G' && incoming_message[5] == 'A' && (incoming_message[44] == '1' || incoming_message[44] == '2')) {
			// Filter the minutes for the GGA line multiplied by 10
			lat_gps_actual = ((int)incoming_message[19] - 48) * (long)10000000;
			lat_gps_actual += ((int)incoming_message[20] - 48) * (long)1000000;
			lat_gps_actual += ((int)incoming_message[22] - 48) * (long)100000;
			lat_gps_actual += ((int)incoming_message[23] - 48) * (long)10000;
			lat_gps_actual += ((int)incoming_message[24] - 48) * (long)1000;
			lat_gps_actual += ((int)incoming_message[25] - 48) * (long)100;
			lat_gps_actual += ((int)incoming_message[26] - 48) * (long)10;
			// To convert minutes to degrees we need to divide minutes by 6
			lat_gps_actual /= (long)6;
			// Add multiply degrees by 10
			lat_gps_actual += ((int)incoming_message[17] - 48) * (long)100000000;
			lat_gps_actual += ((int)incoming_message[18] - 48) * (long)10000000;
			// Divide everything by 10
			lat_gps_actual /= 10;

			// Filter minutes for the GGA line multiplied by 10
			lon_gps_actual = ((int)incoming_message[33] - 48) * (long)10000000;
			lon_gps_actual += ((int)incoming_message[34] - 48) * (long)1000000;
			lon_gps_actual += ((int)incoming_message[36] - 48) * (long)100000;
			lon_gps_actual += ((int)incoming_message[37] - 48) * (long)10000;
			lon_gps_actual += ((int)incoming_message[38] - 48) * (long)1000;
			lon_gps_actual += ((int)incoming_message[39] - 48) * (long)100;
			lon_gps_actual += ((int)incoming_message[40] - 48) * (long)10;
			// To convert minutes to degrees we need to divide minutes by 6
			lon_gps_actual /= (long)6;
			// Add multiply degrees by 10
			lon_gps_actual += ((int)incoming_message[30] - 48) * (long)1000000000;
			lon_gps_actual += ((int)incoming_message[31] - 48) * (long)100000000;
			lon_gps_actual += ((int)incoming_message[32] - 48) * (long)10000000;
			// Divide everything by 10
			lon_gps_actual /= 10;

			if (incoming_message[28] == 'S')
				// South latitude
				lat_gps_actual *= -1;

			if (incoming_message[42] == 'W')
				// West longitude
				lon_gps_actual *= -1;

			// Filter the number of satillites from the GGA line
			number_used_sats = ((int)incoming_message[46] - 48) * (long)10;
			number_used_sats += (int)incoming_message[47] - 48;

			// If this is the first time the GPS code is used
			if (lat_gps_previous == 0 && lon_gps_previous == 0) {
				// Set the lat_gps_previous variable to the lat_gps_actual variable
				lat_gps_previous = lat_gps_actual;
				// Set the lon_gps_previous variable to the lon_gps_actual variable
				lon_gps_previous = lon_gps_actual;
			}

			// Divide the difference between the new and previous positions by ten.
			lat_gps_loop_add = (float)(lat_gps_actual - lat_gps_previous) / 10.0;
			lon_gps_loop_add = (float)(lon_gps_actual - lon_gps_previous) / 10.0;

			// Set the l_lat_gps variable to the previous latitude value
			l_lat_gps = lat_gps_previous;
			// Set the l_lon_gps variable to the previous longitude value
			l_lon_gps = lon_gps_previous;

			// Remember the new position value in the lat_gps_previous variable for the next loop.
			lat_gps_previous = lat_gps_actual;
			lon_gps_previous = lon_gps_actual;

			// The GPS is set to a 5Hz refresh rate. Between every 2 GPS measurments, 9 GPS values are simulated
			// Set the gps_add_counter variable to 5 as a count down loop timer
			gps_add_counter = 5;
			// Set the new_gps_data_counter to 9. This is the number of simulated values between 2 GPS measurements.
			new_gps_data_counter = 9;
			// Reset variables
			lat_gps_add = 0;
			lon_gps_add = 0;
			new_gps_data_available = 1;
		}

		// If the line starts with SA and if there is a GPS fix we can scan the line for the fix type (none, 2D or 3D)
		if (incoming_message[4] == 'S' && incoming_message[5] == 'A')
			fix_type = (int)incoming_message[9] - 48;
	}

	// After 5 program loops 5 x 4ms = 20ms the gps_add_counter is 0.
	if (gps_add_counter == 0 && new_gps_data_counter > 0) {
		// If gps_add_counter is 0 and there are new GPS simulations needed
		// Set the new_gps_data_available to indicate that there is new data available.
		new_gps_data_available = 1;
		// Decrement the new_gps_data_counter so there will only be 9 simulations
		new_gps_data_counter--;
		// Set the gps_add_counter variable to 5 as a count down loop timer
		gps_add_counter = 5;

		// Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers.
		lat_gps_add += lat_gps_loop_add;
		if (abs(lat_gps_add) >= 1) {
			// Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part
			l_lat_gps += (int)lat_gps_add;
			// Subtract the lat_gps_add value as an integer so the decimal value remains
			lat_gps_add -= (int)lat_gps_add;
		}

		// Add the simulated part to a buffer float variable because the l_lat_gps can only hold integers
		lon_gps_add += lon_gps_loop_add;
		if (abs(lon_gps_add) >= 1) {
			// Increment the lat_gps_add value with the lat_gps_add value as an integer. So no decimal part
			l_lon_gps += (int)lon_gps_add;
			// Subtract the lat_gps_add value as an integer so the decimal value remains
			lon_gps_add -= (int)lon_gps_add;
		}
	}
	if (new_gps_data_available) {
		// If there is a new set of GPS data available
		// Change the LED on the STM32 to indicate GPS reception
		if (number_used_sats < 8)
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		else 
			digitalWrite(LED_BUILTIN, 0);

		// Reset the lost watchdog
		gps_lost_counter = 0;

		// Reset the new_gps_data_available flag
		new_gps_data_available = 0;

		if (flight_mode >= 3 && !gps_setpoint_set) {
			// Remember the current location as setpoint if the flight mode is 3 (GPS hold) and no setpoints are set
			gps_setpoint_set = 1;
			l_lat_setpoint = l_lat_gps;
			l_lon_setpoint = l_lon_gps;
			pid_gps_reset();
		}

		if (flight_mode >= 3 && gps_setpoint_set) {
			// If the GPS hold mode and the setpoints are stored.

			if (flight_mode == 3 && takeoff_detected == 1) {
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