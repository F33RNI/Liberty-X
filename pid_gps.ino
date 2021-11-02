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
/// Executes GPS PID controller
/// </summary>
void pid_gps(void) {
	// Calculate the error between setpoint and actual position
	gps_lat_error = l_lat_gps - l_lat_setpoint;
	gps_lon_error = l_lon_setpoint - l_lon_gps;

	// Subtract the current memory position to make room for the new value
	gps_lat_total_avarage -= gps_lat_rotating_mem[gps_rotating_mem_location];
	gps_lon_total_avarage -= gps_lon_rotating_mem[gps_rotating_mem_location];

	// Calculate the new change between the actual position and the previous measurement.
	gps_lat_rotating_mem[gps_rotating_mem_location] = gps_lat_error - gps_lat_error_previous;
	gps_lon_rotating_mem[gps_rotating_mem_location] = gps_lon_error - gps_lon_error_previous;

	// Add the new values to the long term avarage values
	gps_lat_total_avarage += gps_lat_rotating_mem[gps_rotating_mem_location];
	gps_lon_total_avarage += gps_lon_rotating_mem[gps_rotating_mem_location];

	// Increase the rotating memory location
	gps_rotating_mem_location++;
	// Start at 0 when the memory location 35 is reached
	if (gps_rotating_mem_location == 35)
		gps_rotating_mem_location = 0;

	// Remember the errors for the next loop
	gps_lat_error_previous = gps_lat_error;
	gps_lon_error_previous = gps_lon_error;

	// Calculate the GPS PID correction as if the nose of the multicopter is facing north.
	// The Proportional part = (float)gps_lat_error * gps_p_gain.
	// The Derivative part = (float)gps_lat_total_avarage * gps_d_gain.
	pid_output_gps_lat = (float)gps_lat_error * PID_GPS_P + (float)gps_lat_total_avarage * PID_GPS_D;
	pid_output_gps_lon = (float)gps_lon_error * PID_GPS_P + (float)gps_lon_total_avarage * PID_GPS_D;

	// Because the correction is calculated as if the nose was facing north, we need to convert it for the current heading
	gps_pitch_adjust = ((float)pid_output_gps_lat * cos(angle_yaw * DEG_TO_RAD)) + ((float)pid_output_gps_lon * cos((angle_yaw + 90) * DEG_TO_RAD));
	gps_roll_adjust = ((float)pid_output_gps_lon * cos(angle_yaw * DEG_TO_RAD)) + ((float)pid_output_gps_lat * cos((angle_yaw - 90) * DEG_TO_RAD));

	// Clip PID output
	if (gps_pitch_adjust > PID_GPS_MAX) gps_pitch_adjust = PID_GPS_MAX;
	if (gps_pitch_adjust < -PID_GPS_MAX) gps_pitch_adjust = -PID_GPS_MAX;
	if (gps_roll_adjust > PID_GPS_MAX) gps_roll_adjust = PID_GPS_MAX;
	if (gps_roll_adjust < -PID_GPS_MAX) gps_roll_adjust = -PID_GPS_MAX;
}

/// <summary>
/// Resets GPS PID controller
/// </summary>
void pid_gps_reset(void) {
	// Reset GPS PID controllers
	gps_lat_error_previous = 0;
	gps_lon_error_previous = 0;
	gps_lat_total_avarage = 0;
	gps_lon_total_avarage = 0;
	gps_rotating_mem_location = 0;

	// Reset output corrections
	gps_roll_adjust = 0;
	gps_pitch_adjust = 0;
}
