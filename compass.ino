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
/// Initializes the compass
/// </summary>
void compass_setup(void) {
	// Check if the compass is responding
	HWire.beginTransmission(COMPASS_ADDRESS);
	error = HWire.endTransmission();
	while (error != 0) {
		// Stay in the loop because the compass did not responde
		error = 2;
		// Show cuurent error
		leds_error_signal();
		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Start communication with the compass
	HWire.beginTransmission(COMPASS_ADDRESS);

	// Write to the Configuration Register A (00 hex)
	HWire.write(0x00);

	// Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz)
	HWire.write(0x78);

	// Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga
	HWire.write(0x20);

	// Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode
	HWire.write(0x00);

	// End the transmission with the compass
	HWire.endTransmission();

	// Read the calibration values from the EEPROM
	for (count_var = 0; count_var < 6; count_var++)
		compass_cal_values[count_var] = EEPROM.read(0x10 + count_var);

	// Calculate the calibration offset and scale values
	compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
	compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

	compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
	compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
	compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;

	// Read current data
	compass_read();
}

/// <summary>
/// Reads and calculates heading from the compass
/// </summary>
void compass_read() {
	// Start reading at the hexadecimal location 0x03
	HWire.beginTransmission(COMPASS_ADDRESS);
	HWire.write(0x03);
	HWire.endTransmission();

	// Request 6 bytes from the compass and read all axes
	HWire.requestFrom(COMPASS_ADDRESS, 6);
	compass_y = HWire.read() << 8 | HWire.read();
	compass_y *= -1;
	compass_z = HWire.read() << 8 | HWire.read();
	compass_x = HWire.read() << 8 | HWire.read();
	compass_x *= -1;

	if (!compass_calibration_flag) {
		// Subtact the compass calibration values
		compass_y += compass_offset_y;
		compass_y *= compass_scale_y;
		compass_z += compass_offset_z;
		compass_z *= compass_scale_z;
		compass_x += compass_offset_x;
	}

	// The compass values change when the roll and pitch angle of the quadcopter changes
	compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) 
		+ (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) 
		- (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
	compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) 
		+ (float)compass_z * sin(angle_roll * 0.0174533);

	if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
	else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

	// Add the declination to the magnetic compass heading to get the geographic north
	actual_compass_heading += COMPASS_DECLINATION;             

	// Keep 0 - 360 range
	if (actual_compass_heading < 0)
		actual_compass_heading += 360;
	else if (actual_compass_heading >= 360)
		actual_compass_heading -= 360;
}


/// <summary>
/// Calibrates compass
/// </summary>
void compass_calibrate(void) {
	// Disable subtracting calibration values
	compass_calibration_flag = 1;

	// Print old values to the serial port if needed
#ifdef PRINT_COMPASS_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibrating compass..."));
	TELEMETRY_SERIAL.println(F("Old values:"));
	for (count_var = 0; count_var < 6; count_var++) {
		TELEMETRY_SERIAL.print(count_var);
		TELEMETRY_SERIAL.print('\t');
		TELEMETRY_SERIAL.println(compass_cal_values[count_var]);
	}
#endif

	// Reset old values
	for (count_var = 0; count_var < 6; count_var++)
		compass_cal_values[count_var] = 0;

	count_var = 0;
	while (channel_2 < 1900) {
		// Stay in this loop until the pilot lowers the pitch stick of the transmitter
		// Read the raw compass values
		compass_read();

		// Store the maximum and minimum detected compass values
		if (compass_x < compass_cal_values[0])
			compass_cal_values[0] = compass_x;
		if (compass_x > compass_cal_values[1])
			compass_cal_values[1] = compass_x;

		if (compass_y < compass_cal_values[2])
			compass_cal_values[2] = compass_y;
		if (compass_y > compass_cal_values[3])
			compass_cal_values[3] = compass_y;

		if (compass_z < compass_cal_values[4])
			compass_cal_values[4] = compass_z;
		if (compass_z > compass_cal_values[5])
			compass_cal_values[5] = compass_z;

		// Blick with the LEDs
		count_var++;
		if (count_var >= 125) {
			leds_calibration_signal();
			count_var = 0;
		}

		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Print new values to the serial port if needed
#ifdef PRINT_COMPASS_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibration done"));
	TELEMETRY_SERIAL.println(F("New values:"));
	for (count_var = 0; count_var < 6; count_var++) {
		TELEMETRY_SERIAL.print(count_var);
		TELEMETRY_SERIAL.print('\t');
		TELEMETRY_SERIAL.println(compass_cal_values[count_var]);
	}
#endif

	// Enable subtracting calibration values
	compass_calibration_flag = 0;

	// Store the maximum and minimum values to the EEPROM
	for (count_var = 0; count_var < 6; count_var++)
		EEPROM.write(0x10 + count_var, compass_cal_values[count_var]);

	// Re-initialize compass
	compass_setup();

	// Set the initial compass heading
	angle_yaw = actual_compass_heading;

	// Reset loop_timer
	loop_timer = micros();
}

/// <summary>
/// Calculates the smallest difference between two heading values
/// </summary>
/// <param name="course_b"> First heading values </param>
/// <param name="course_c"> Second heading value </param>
/// <returns> Difference between two heading values </returns>
float course_deviation(float course_b, float course_c) {
	course_a = course_b - course_c;
	if (course_a < -180 || course_a > 180) {
		if (course_c > 180)base_course_mirrored = course_c - 180;
		else base_course_mirrored = course_c + 180;
		if (course_b > 180)actual_course_mirrored = course_b - 180;
		else actual_course_mirrored = course_b + 180;
		course_a = actual_course_mirrored - base_course_mirrored;
	}
	return course_a;
}
