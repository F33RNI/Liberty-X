/*
 * Copyright (C) 2022 Fern Lane, Liberty-X Flight controller
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
 * The Liberty-X project started as a fork of the YMFC-32 project by Joop Brokking
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
	while (error) {
		// Stay in the loop because the sonarus did not responde
		error = ERROR_BOOT_SONARUS;
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

#ifdef SONARUS_COLLISION_PROTECTION
	// Clear collision_protection_started flag if drone is not in flight
	if (!takeoff_detected || start < 2)
		collision_protection_started = 0;
#endif

	// Last cycle. Request LUX data
	if (sonarus_cycle_counter >= SONARUS_REQUST_CYCLES) {
		// Restart counter
		sonarus_cycle_counter = 0;

		// Request 4 bytes from sonarus (2 bytes per sonar)
		HWire.requestFrom(SONARUS_ADDRESS, 4);

		// Read distance from first sonar
		sonarus_front = HWire.read() << 8 | HWire.read();

		// Read distance from second sonar
		sonarus_bottom = HWire.read() << 8 | HWire.read();

		// Convert 2 distance to cm/2
		sonarus_bottom_compressed = sonarus_bottom / 20;
		if (sonarus_bottom_compressed > 255)
			sonarus_bottom_compressed = 255;

		// Calculate loop_add for sonarus predictions
		sonarus_bottom_loop_add = ((float)sonarus_bottom - (float)sonarus_bottom_previous) / SONARUS_REQUST_CYCLES;

		// Store old second sonar value
		sonarus_bottom_previous = sonarus_bottom;
		
#ifdef SONARUS_COLLISION_PROTECTION
		// Increment or decrement collision protection counter
		if (sonarus_front > 0 && sonarus_front <= SONARUS_COLLISION_PROTECTION_START) {
			if (collision_protection_counter < SONARUS_COLLISION_PROTECTION_CYCLES)
				collision_protection_counter++;
		}
		else if (collision_protection_counter > 0)
			collision_protection_counter--;

		// Execute collision protection
		if (start > 1 && takeoff_detected && !auto_landing_step && !collision_protection_started
			&& collision_protection_counter >= SONARUS_COLLISION_PROTECTION_CYCLES)
			sonarus_start_collision_protection();

		// Collision protection PD controller
		sonarus_collision_protection();
#endif
	}

	// Sonarus predictions
	else {
		// Add the simulated part to a buffer float variables because the sonarus_bottom can only hold integers
		sonarus_bottom_add += sonarus_bottom_loop_add;

		// If the absolute value of sonarus_bottom_add is larger then 1
		if (abs(sonarus_bottom_add) >= 1) {
			// Prevent overflowing (going below zero)
			if ((int)sonarus_bottom + (int)sonarus_bottom_add >= 0) {
				// Increment the sonarus_bottom value with the sonarus_bottom_add value as an integer
				sonarus_bottom += (int)sonarus_bottom_add;

				// Subtract the sonarus_bottom_add value as an integer so the decimal value remains
				sonarus_bottom_add -= (int)sonarus_bottom_add;
			}
			else
				sonarus_bottom = 0;
		}
	}
}

/// <summary>
/// Executes 2nd sonar PID controller
/// </summary>
/// <param name=""></param>
void sonarus_pid(void) {
	// Execute Sonarus PID controller only if sonarus_bottom is not zero
	if (sonarus_bottom > 0) {
		// Disable pressure control
		pid_alt_setpoint = actual_pressure;

		pid_error_temp = pid_sonarus_setpoint - (float)sonarus_bottom;

		pid_i_mem_sonarus += PID_SONARUS_I * pid_error_temp;
		if (pid_i_mem_sonarus > PID_SONARUS_MAX)pid_i_mem_sonarus = PID_SONARUS_MAX;
		else if (pid_i_mem_sonarus < PID_SONARUS_MAX * -1)pid_i_mem_sonarus = PID_SONARUS_MAX * -1;

		pid_output_sonarus = PID_SONARUS_P * pid_error_temp + pid_i_mem_sonarus + PID_SONARUS_D * (pid_error_temp - pid_last_sonarus_d_error);
		if (pid_output_sonarus > PID_SONARUS_MAX)pid_output_sonarus = PID_SONARUS_MAX;
		else if (pid_output_sonarus < PID_SONARUS_MAX * -1)pid_output_sonarus = PID_SONARUS_MAX * -1;

		pid_last_sonarus_d_error = pid_error_temp;
	}
	
	// Reset sonarus PID controller in case of sonarus lost
	else
		sonarus_pid_reset();
}

/// <summary>
/// Resets bottom sonarus PID controller
/// </summary>
/// <param name=""></param>
void sonarus_pid_reset(void) {
	pid_output_sonarus = 0;
	pid_i_mem_sonarus = 0;
	pid_last_sonarus_d_error = 0;
}

#ifdef SONARUS_COLLISION_PROTECTION
/// <summary>
/// Start collision protection and auto-landing sequence
/// </summary>
/// <param name=""></param>
void sonarus_start_collision_protection(void) {
	// Show error
	error = ERROR_SONARUS_COLLISION;

	// Start auto-landing
	auto_landing_step = 1;

	// Start PD-controller
	collision_protection_started = 1;
}

/// <summary>
/// Brings the drone back
/// </summary>
/// <param name=""></param>
void sonarus_collision_protection(void) {
	// Execute PD controller
	if (collision_protection_started && sonarus_front > 0 && sonarus_front <= SONARUS_COLLISION_PROTECTION_START) {
		pid_error_temp = (float)SONARUS_COLLISION_PROTECTION_START - (float)sonarus_front;

		collision_protection_pitch = SONARUS_PROTECTION_P * pid_error_temp + SONARUS_PROTECTION_D * (pid_error_temp - sonarus_front_d_error);

		if (collision_protection_pitch > SONARUS_PROTECTION_MAX)
			collision_protection_pitch = SONARUS_PROTECTION_MAX;
		else if (collision_protection_pitch < SONARUS_PROTECTION_MAX * -1)
			collision_protection_pitch = SONARUS_PROTECTION_MAX * -1;

		sonarus_front_d_error = pid_error_temp;
	}

	// Reset PD controller if the distance is normal
	else {
		collision_protection_pitch = 0;
		sonarus_front_d_error = 0;
	}
}
#endif
#endif
