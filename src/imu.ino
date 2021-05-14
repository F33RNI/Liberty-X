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
/// Initializes MPU-6050 IMU
/// </summary>
void imu_setup(void) {
	// Check if the IMU is responding
	HWire.beginTransmission(IMU_ADDRESS);
	error = HWire.endTransmission();
	while (error != 0) {
		// Stay in the loop because the IMU did not responde
		error = 1;
		// Show cuurent error
		leds_error_signal();
		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Set the PWR_MGMT_1 register (6B hex) bits as 00000000 to activate the gyro
	HWire.beginTransmission(IMU_ADDRESS);
	HWire.write(0x6B);
	HWire.write(0x00);
	HWire.endTransmission();

	// Set the GYRO_CONFIG register (1B hex) bits as 00001000 (500dps full scale)
	HWire.beginTransmission(IMU_ADDRESS);
	HWire.write(0x1B);
	HWire.write(0x08);
	HWire.endTransmission();

	// Set the  ACCEL_CONFIG register (1A hex) bits as 00010000 (+/- 8g full scale range)
	HWire.beginTransmission(IMU_ADDRESS);
	HWire.write(0x1C);
	HWire.write(0x10);
	HWire.endTransmission();

	// Set the CONFIG register (1A hex) bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	HWire.beginTransmission(IMU_ADDRESS);
	HWire.write(0x1A);
	HWire.write(0x03);
	HWire.endTransmission();

	// Set manual calibration values or from the EEPROM
#ifdef ACC_CALIBRATION_PITCH
	acc_pitch_cal = ACC_CALIBRATION_PITCH;
#else
	acc_pitch_cal = EEPROM.read(0x16);
#endif
#ifdef ACC_CALIBRATION_ROLL
	acc_roll_cal = ACC_CALIBRATION_ROLL;
#else
	acc_roll_cal = EEPROM.read(0x17);
#endif
}

/// <summary>
/// Reads raw data from the IMU with calibrartions
/// </summary>
void imu_read(void) {
	HWire.beginTransmission(IMU_ADDRESS);
	// Start reading @ register 43h and auto increment with every read()
	HWire.write(0x3B);
	HWire.endTransmission();

	// Request 14 bytes from the MPU 6050.
	HWire.requestFrom(IMU_ADDRESS, 14);

	// Add the low and high byte to the acc variables
	acc_y = HWire.read() << 8 | HWire.read();
	acc_x = HWire.read() << 8 | HWire.read();
	acc_z = HWire.read() << 8 | HWire.read();

	// Add the low and high byte to the temperature variable
	temperature = HWire.read() << 8 | HWire.read();

	// Read high and low parts of the angular data
	gyro_roll = HWire.read() << 8 | HWire.read();
	gyro_pitch = HWire.read() << 8 | HWire.read();
	gyro_yaw = HWire.read() << 8 | HWire.read();

	// Invert the direction of the axes
	//gyro_roll *= -1;
	gyro_pitch *= -1;
	gyro_yaw *= -1;

	if (!acc_calibration_flag) {
		// Subtact the acc calibration values
		acc_x -= acc_roll_cal;
		acc_y -= acc_pitch_cal;
	}

	if (!gyro_calibration_flag) {
		// Subtact the gyro calibration values
		gyro_roll -= gyro_roll_cal;
		gyro_pitch -= gyro_pitch_cal;
		gyro_yaw -= gyro_yaw_cal;
	}
}

/// <summary>
/// Calibartes gyro
/// </summary>
void imu_calibrate_gyro(void) {
	// Disable subtracting calibration values
	gyro_calibration_flag = 1;

	// Print to the serial port if needed
#ifdef PRINT_GYRO_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibrating gyro..."));
#endif

	// Wait some time before calibration
	for (count_var = 0; count_var < 1000; count_var++) {
		// Blink with LEDs
		if (count_var % 125 == 0)
			leds_calibration_signal();

		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Reset gyro calibration values
	gyro_roll_cal = 0;
	gyro_pitch_cal = 0;
	gyro_yaw_cal = 0;
	for (count_var = 0; count_var < IMU_CALIBARTION_N; count_var++) {
		// Read data from the IMU
		imu_read();

		// Store values
		gyro_roll_cal += gyro_roll;
		gyro_pitch_cal += gyro_pitch;
		gyro_yaw_cal += gyro_yaw;

		// Blink with LEDs
		if (count_var % 25 == 0)
			leds_calibration_signal();

		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}
	// Devide to get average calibration value
	gyro_roll_cal /= IMU_CALIBARTION_N;
	gyro_pitch_cal /= IMU_CALIBARTION_N;
	gyro_yaw_cal /= IMU_CALIBARTION_N;

	// Print to the serial port if needed
#ifdef PRINT_GYRO_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibration done"));
	TELEMETRY_SERIAL.println(F("New values:"));
	TELEMETRY_SERIAL.print(F("Roll: "));
	TELEMETRY_SERIAL.println(gyro_roll_cal);
	TELEMETRY_SERIAL.print(F("Pitch: "));
	TELEMETRY_SERIAL.println(gyro_pitch_cal);
	TELEMETRY_SERIAL.print(F("Yaw: "));
	TELEMETRY_SERIAL.println(gyro_yaw_cal);
#endif

	// Enable subtracting calibration values
	gyro_calibration_flag = 0;

	// Reset loop_timer
	loop_timer = micros();
}

/// <summary>
/// Calibartes level
/// </summary>
void imu_calibrate_acc(void) {
	// Disable subtracting calibration values
	acc_calibration_flag = 1;

	// Print to the serial port if needed
#ifdef PRINT_LEVEL_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibrating level..."));
#endif

	// Reset gyro calibration values
	acc_roll_cal = 0;
	acc_pitch_cal = 0;
	for (count_var = 0; count_var < IMU_CALIBARTION_N; count_var++) {
		// Read data from the IMU
		imu_read();

		// Store values
		acc_roll_cal += acc_x;
		acc_pitch_cal += acc_y;

		// Blink with LEDs
		if (count_var % 25 == 0)
			leds_calibration_signal();

		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}
	// Devide to get average calibration value
	acc_roll_cal /= IMU_CALIBARTION_N;
	acc_pitch_cal /= IMU_CALIBARTION_N;

	// Print to the serial port if needed
#ifdef PRINT_LEVEL_CALIBRATION
	TELEMETRY_SERIAL.println(F("Calibration done"));
	TELEMETRY_SERIAL.println(F("New values:"));
	TELEMETRY_SERIAL.print(F("Roll: "));
	TELEMETRY_SERIAL.println(acc_roll_cal);
	TELEMETRY_SERIAL.print(F("Pitch: "));
	TELEMETRY_SERIAL.println(acc_pitch_cal);
#endif

	// Enable subtracting calibration values
	acc_calibration_flag = 0;

	// Store values to the EEPROM
	EEPROM.write(0x16, acc_pitch_cal);
	EEPROM.write(0x17, acc_roll_cal);

	// Read data from the IMU
	imu_read();

	// Reset loop_timer
	loop_timer = micros();
}