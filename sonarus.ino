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

#ifdef SONARUS

/// <summary>
/// Initilizes
/// </summary>
/// <param name=""></param>
void sonarus_setup(void) {
	// Check if the sonar system is responding
	HWire.beginTransmission(SONARUS_ADDRESS);
	error = HWire.endTransmission();
	while (error != 0) {
		// Stay in the loop because the sonarus did not responde
		error = 5;
		// Show curent error
		leds_error_signal();
		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Power on sonarus
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x01);
	HWire.endTransmission();

	// Select continuously mode
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x02);
	HWire.write(0x01);
	HWire.endTransmission();

	// Select high-resolution (2-byte) mode (distances will be in mm)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x03);
	HWire.write(0x01);
	HWire.endTransmission();

	// Reset filter
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x04);
	HWire.write(0x00);
	HWire.endTransmission();

	// Set speed of sound
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x05);
	HWire.write(SOUND_SPEED >> 8);
	HWire.write(SOUND_SPEED & 0xFF);
	HWire.endTransmission();

	// Set distance correction for first sonar (-34.3) (-343 -> 0xFEA4)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x06);
	HWire.write(0xFE);
	HWire.write(0xA9);
	HWire.write(0x00);
	HWire.write(0x00);
	HWire.endTransmission();

	// Request first measurement
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x07);
	HWire.endTransmission();

	// Wait for first measurement to complete
	delay(100);

	// Set filter (default: 0.4 -> 40)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x04);
	HWire.write(40);
	HWire.endTransmission();
}

void sonarus(void) {
	// Increment counter every cycle
	sonarus_cycle_counter++;

	// Last cycle. Request LUX data
	if (sonarus_cycle_counter >= SONARUS_REQUST_CYCLES) {
		// Restart counter
		sonarus_cycle_counter = 0;

		// Store previous value
#ifdef SONARUS_TAKEOFF_DETECTION
		sonar_2_prev = sonar_2_raw;
#endif

		// Request 4 bytes from sonarus (2 bytes per sonar)
		HWire.requestFrom(SONARUS_ADDRESS, 4);

		// Read distance from first sonar
		sonar_1_raw = HWire.read() << 8 | HWire.read();

		// Read distance from second sonar
		sonar_2_raw = HWire.read() << 8 | HWire.read();
	}
}

/// <summary>
/// Executes 2nd sonar PID controller
/// </summary>
/// <param name=""></param>
void sonarus_pid(void) {
	// Disable pressure control
	pid_alt_setpoint = actual_pressure;

	// Execute Sonarus PID controller only if sonarus_cycle_counter is 1
	if (sonarus_cycle_counter == 1) {
		pid_error_temp = pid_sonar_setpoint - (float)sonar_2_raw;

		pid_i_mem_sonar += PID_SONARUS_I * pid_error_temp;
		if (pid_i_mem_sonar > PID_SONARUS_MAX)pid_i_mem_sonar = PID_SONARUS_MAX;
		else if (pid_i_mem_sonar < PID_SONARUS_MAX * -1)pid_i_mem_sonar = PID_SONARUS_MAX * -1;

		pid_output_sonar = PID_SONARUS_P * pid_error_temp + pid_i_mem_sonar + PID_SONARUS_D * (pid_error_temp - pid_last_sonar_d_error);
		if (pid_output_sonar > PID_SONARUS_MAX)pid_output_sonar = PID_SONARUS_MAX;
		else if (pid_output_sonar < PID_SONARUS_MAX * -1)pid_output_sonar = PID_SONARUS_MAX * -1;

		pid_last_sonar_d_error = pid_error_temp;

		// If a difference of 20 cm is reached, current command is >= 100 (4) and waypoints_index less than 15
		if (pid_error_temp < 200 && waypoints_command[waypoints_index] >= 0b100 && waypoints_index < 15) {
			// Switch to next waypoint in an array
			waypoints_index++;

			// Incrememnt altitude
			link_waypoint_step = 2;
		}

	}
}
#endif
