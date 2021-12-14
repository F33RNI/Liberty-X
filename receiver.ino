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
/// Pre-flight calibartions and programming mode
/// </summary>
void receiver_pre_flight(void) {
	if (start == 0) {
		// Run some calibrations or disable ESC output before takeoff
		if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900)
			// Top right. Compass calibration
			compass_calibrate();

		if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100)
			// Top left. Level calibration
			imu_calibrate_acc();

		if (channel_1 < 1100 && channel_2 > 1900 && channel_3 < 1100 && channel_4 < 1100) {
			// Bottom left. Disable drone (programming mode)
			while (channel_2 > 1100)
			{
				// Stay in this loop until the pilot rises the pitch stick of the transmitter

				// Disable the LEDs
				ws_leds.clear();
				ws_leds.show();

				// Disable the motors
				TIMER4_BASE->CCR1 = 0;
				TIMER4_BASE->CCR2 = 0;
				TIMER4_BASE->CCR3 = 0;
				TIMER4_BASE->CCR4 = 0;
				TIMER4_BASE->CNT = 5000;

				// Disable gimbal
				TIMER3_BASE->CCR4 = 0;
				TIMER3_BASE->CNT = 5000;

				// Simulate main loop
				delayMicroseconds(LOOP_PERIOD);
			}
			// Reset loop timer
			loop_timer = micros();
		}
	}
}

/// <summary>
/// Selects flight modes with the receiver
/// </summary>
void receiver_modes(void) {
	// Flight modes
	if (channel_5 > 1700) {
		flight_mode = 3;
	}
	else if (channel_5 > 1300) {
		flight_mode = 2;
	}

#ifdef LIBERTY_LINK
	// Enable Liberty-Link if channel_7 > 1500
	if (channel_7 > 1500)
		link_allowed = 1;
	else
		link_allowed = 0;
#else
	// Enable heading lock if channel_7 > 1500
	//if (channel_7 > 1500)
	//	heading_lock_enabled = 1;
	//else
		heading_lock_enabled = 0;
#endif

	// Turn off all corrections
	if (channel_5 < 1300) {
		flight_mode = 1;
		heading_lock_enabled = 0;
	}
}

/// <summary>
/// Starts or stops the motors and executes the takeoff sequence
/// </summary>
void receiver_start_stop(void) {
	// Pre-start the motors (step 1)
	if (start == 0 && channel_3 < 1050 && channel_6 > 1500) {
		// Switch to the step 2 if throttle stick is in lowest position and arm switch is on
		start = 1;

#ifdef LIBERTY_LINK
		if (link_takeoff_flag) {
			// Reset takeoff flag
			link_takeoff_flag = 0;

			// Check sonarus
#ifdef SONARUS
			if (sonarus_bottom == 0 || sonarus_bottom > SONARUS_DESCENT_MM) {
				start = 0;
				error = 9;
			}
#endif
			// Execute FTS if error occurs
			if (error)
				liberty_x_fts();
		}
#endif
	}

	// Record ground pressure
	if (start < 2)
		ground_pressure = actual_pressure;

	// Start the motors (step 2)
	if (start == 1 && channel_3 > 1050 && channel_6 > 1500) {
		// Start the motors if throttle stick is not in lowest position and arm switch is on
		start = 2;

		// Set current GPS position as setpoint
		l_lat_setpoint = l_lat_gps;
		l_lon_setpoint = l_lon_gps;

		// Reset GPS corrections
		l_lat_gps_float_adjust = 0;
		l_lon_gps_float_adjust = 0;
#ifdef LIBERTY_LINK
		waypoint_move_factor = 0;
#endif
		pid_gps_reset();

		// Reset some variables
		throttle = MOTOR_IDLE_SPEED;
		angle_pitch = angle_pitch_acc;
		angle_roll = angle_roll_acc;
		course_lock_heading = angle_yaw;
		acc_total_vector_at_start = acc_total_vector;
#if defined(LIBERTY_LINK) && defined(SONARUS)
		sonarus_pid_reset();
#endif
		acc_alt_integrated = 0;

		if (MANUAL_TAKEOFF_THROTTLE > 1100 && MANUAL_TAKEOFF_THROTTLE < 1700) {
			// If the manual hover throttle is used and valid (between 1100 and 1700)
			// Use the manual hover throttle
			takeoff_throttle = MANUAL_TAKEOFF_THROTTLE - 1500;
			takeoff_detected = 1;

			// Reset the PID controllers for a smooth take-off
			pid_roll_pitch_yaw_reset();
			pid_altitude_reset();
			pid_gps_reset();
#ifdef LIBERTY_LINK
#ifdef SONARUS
			sonarus_pid_reset();
#endif
			l_lat_gps_float_adjust = 0;
			l_lon_gps_float_adjust = 0;
			waypoint_move_factor = 0;
			link_begin_sequence();
#endif

			// Raise altitude to some point
			pid_alt_setpoint = ground_pressure - PRESSURE_TAKEOFF;
		}
		else if (MANUAL_TAKEOFF_THROTTLE) {
			// If the manual hover throttle is not valid
			error = 5;
			takeoff_throttle = 0;
			start = 0;

			// Execute FTS
			liberty_x_fts();
		}
	}

	// Stop the motors if the arm switch is off
	if (channel_6 < 1500) {
		start = 0;
		takeoff_detected = 0;

		// Reset liberty-link variables
#ifdef LIBERTY_LINK
		link_clear_disarm();
#endif
	}

	if (!takeoff_detected && start == 2) {
		// When the quadcopter is started and no take-off is detected
		if (channel_3 > 1480 && throttle < 1750 && !error) {
			// When the throttle stick is half way or higher and no errors, increase the throttle
			throttle++;
		}
		if (throttle == 1750) {
			// If take-off is not detected when the throttle has reached 1700: error = 6 and lower the throttle
			error = 6;
			//throttle = MOTOR_IDLE_SPEED;
		}
		if (channel_3 <= 1480) {
			// When the throttle is below the center stick position
			// Lower the throttle to the motor_idle_speed variable
			if (throttle > MOTOR_IDLE_SPEED)
				throttle--;
			
			// Reset the PID controllers for smooth takeoff
			pid_roll_pitch_yaw_reset();
			pid_altitude_reset();
			pid_gps_reset();
#if defined(LIBERTY_LINK) && defined(SONARUS)
			sonarus_pid_reset();
#endif
		}

		if (acc_z_average_short_total / 25 - acc_total_vector_at_start > AUTO_TAKEOFF_ACC_THRESHOLD) {
			// A take-off is detected when the quadcopter is accelerating
			// Set the take-off detected variable to 1 to indicate a take-off
			takeoff_detected = 1;

			// Start Liberty-Way sequence
#ifdef LIBERTY_LINK
			link_begin_sequence();
#endif

			// Set the altitude setpoint
			pid_alt_setpoint = ground_pressure - PRESSURE_TAKEOFF;

			if (throttle > 1400 && throttle < 1700) {
				// If the automated throttle is between 1400 and 1600us during take-off, calculate take-off throttle
				takeoff_throttle = throttle - 1580; //1530
			}
			else {
				// No take-off throttle is calculated if the automated throttle is not between 1400 and 1700 during take-off
				takeoff_throttle = 0;
				error = 7;
			}
		}

		// Reduce throttle to a full stop if an error occurs during takeoff
		// For only takeoff errors: if (error == 6 || error == 7)
		if (error) {
			takeoff_throttle = 0;
			takeoff_detected = 0;
			if (throttle > 0) {
				throttle--;
			}
			else {
				start = 0;

				// Reset liberty-link variables
#ifdef LIBERTY_LINK
				link_clear_disarm();
#endif

			}
		}

		// Reset link step after flight
#ifdef LIBERTY_LINK
		if (start == 0 && link_waypoint_step > 2)
			link_waypoint_step = 0;
#endif
	}
}

/// <summary>
/// Decodes PPM signal into separate channels
/// </summary>
void ppm_decoder(void) {
	measured_time = TIMER2_BASE->CCR1 - measured_time_start;
	if (measured_time < 0)measured_time += 0xFFFF;
	measured_time_start = TIMER2_BASE->CCR1;
	if (measured_time > 3000)channel_select_counter = 0;
	else channel_select_counter++;

	if (channel_select_counter == 1)
		channel_1 = measured_time;
	if (channel_select_counter == 2)
		channel_2 = measured_time;
	if (channel_select_counter == 3)
		channel_3 = measured_time;
	if (channel_select_counter == 4)
		channel_4 = measured_time;
	if (channel_select_counter == 5)
		channel_5 = measured_time;
	if (channel_select_counter == 6)
		channel_6 = measured_time;
	if (channel_select_counter == 7)
		channel_7 = measured_time;
	if (channel_select_counter == 8)
		channel_8 = measured_time;
}
