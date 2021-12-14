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
/// Collects throttle value and pushes the ESC outputs to the timer
/// </summary>
void throttle_and_motors(void) {
	if (start == 2) {
		// If the drone is in flight

		if (takeoff_detected == 1) {
			// Trottle calculations if takeoff detected
			// Calculate exponential rate for smooth manual control. You can visualize this function in Excel
			throttle_exp = channel_3;
			if (throttle_exp > 1500) {
				// More than 1500
				throttle_exp = 1500.0 + exp((float)abs(throttle_exp - 1500) / MANUAL_TROTTLE_EXP) * MANUAL_TROTTLE_RATE;
			}
			else if (throttle_exp < 1500) {
				// Less than 1500
				throttle_exp = 1500.0 - exp((float)abs(throttle_exp - 1500) / MANUAL_TROTTLE_EXP) * MANUAL_TROTTLE_RATE;
			}

			if (flight_mode >= 2) {
				// Altitude hold
#if (defined(SONARUS) && defined(LIBERTY_LINK))
				if (link_allowed && (link_waypoint_step == LINK_STEP_SONARUS || link_waypoint_step == LINK_STEP_AFTER_SONARUS) && sonarus_bottom > 0)
					throttle = 1500 + takeoff_throttle + pid_output_sonarus;
				else
					throttle = 1500 + takeoff_throttle + pid_output_alt;
#else
				throttle = 1500 + takeoff_throttle + pid_output_alt;
#endif
			}
			else
				// 1 flight mode
				throttle = throttle_exp + takeoff_throttle;

#ifdef LIBERTY_LINK
			if (link_direct_control)
					// Add direct throttle control in direct control mode
					throttle += (direct_throttle_control - 1500);
#endif
		}

		// Limit the throttle
		if (throttle > 1800)
			throttle = 1800;

		// Calculate the pulse for esc 1 (front-right - CCW)
		esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;

		// Calculate the pulse for esc 2 (rear-right - CW)
		esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;

		// Calculate the pulse for esc 3 (rear-left - CCW)
		esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;

		// Calculate the pulse for esc 4 (front-left - CW)
		esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

		// Compensate battery coltage drop
		if (battery_voltage < 12.40 && battery_voltage > 6.0) {
			esc_1 += (12.40 - battery_voltage) * BATTERY_COMPENSATION;
			esc_2 += (12.40 - battery_voltage) * BATTERY_COMPENSATION;
			esc_3 += (12.40 - battery_voltage) * BATTERY_COMPENSATION;
			esc_4 += (12.40 - battery_voltage) * BATTERY_COMPENSATION;
		}

		// Clip the ESC values
		if (esc_1 < MOTOR_IDLE_SPEED) esc_1 = MOTOR_IDLE_SPEED;
		else if (esc_1 > 2000) esc_1 = 2000;
		if (esc_2 < MOTOR_IDLE_SPEED) esc_2 = MOTOR_IDLE_SPEED;
		else if (esc_2 > 2000) esc_2 = 2000;
		if (esc_3 < MOTOR_IDLE_SPEED) esc_3 = MOTOR_IDLE_SPEED;
		else if (esc_3 > 2000) esc_3 = 2000;
		if (esc_4 < MOTOR_IDLE_SPEED) esc_4 = MOTOR_IDLE_SPEED;
		else if (esc_4 > 2000) esc_4 = 2000;
	}
	else {
		// If the drone is not in flight
		esc_1 = 1000;
		esc_2 = 1000;
		esc_3 = 1000;
		esc_4 = 1000;
	}

#ifndef DISABLE_MOTORS
	TIMER4_BASE->CCR1 = esc_1;
	TIMER4_BASE->CCR2 = esc_2;
	TIMER4_BASE->CCR3 = esc_3;
	TIMER4_BASE->CCR4 = esc_4;
#else
	// Send zero speed regardless of esc calculations if DISABLE_MOTORS is defined
	TIMER4_BASE->CCR1 = 1000;
	TIMER4_BASE->CCR2 = 1000;
	TIMER4_BASE->CCR3 = 1000;
	TIMER4_BASE->CCR4 = 1000;
#endif
	TIMER4_BASE->CNT = 5000;
}
