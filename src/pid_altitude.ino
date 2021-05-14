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
/// Executes altitude PID controller
/// </summary>
void pid_altitude(void) {
	// Calculate the error between setpoint and actual position
	pid_error_temp = actual_pressure - pid_alt_setpoint;

	// Subtract the current memory position to make room for the new value
	alt_total_avarage -= alt_rotating_mem[alt_rotating_mem_location];

	// Calculate the new change between the actual pressure and the previous measurement
	alt_rotating_mem[alt_rotating_mem_location] = actual_pressure - alt_total_previous;

	// Add the new value to the long term avarage value
	alt_total_avarage += alt_rotating_mem[alt_rotating_mem_location];

	//Increase the rotating memory location
	alt_rotating_mem_location++;
	// Start at 0 when the memory location 30 is reached
	if (alt_rotating_mem_location == 30)
		alt_rotating_mem_location = 0;

	// Remember the actual pressure for the next loop
	alt_total_previous = actual_pressure;

	// To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases
	// The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller
	pid_error_gain_altitude = 0;
	if (pid_error_temp > 10 || pid_error_temp < -10) {
		// If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10
		// The positive pid_error_gain_altitude variable is calculated based based on the error.
		pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
		// Clip to prevent extreme P-gains
		if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;
	}

	// I-output is calculation. It's an accumulation of errors over time
	pid_i_mem_alt += PID_ALT_I * pid_error_temp;

	// Clip I controller
	if (pid_i_mem_alt > PID_ALT_MAX) pid_i_mem_alt = PID_ALT_MAX;
	else if (pid_i_mem_alt < PID_ALT_MAX * -1) pid_i_mem_alt = PID_ALT_MAX * -1;

	// Calculate output of the PID-controller
	pid_output_alt = (PID_ALT_P + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_alt + alt_total_avarage * PID_ALT_D;

	// Clip PID output
	if (pid_output_alt > PID_ALT_MAX) pid_output_alt = PID_ALT_MAX;
	else if (pid_output_alt < PID_ALT_MAX * -1) pid_output_alt = PID_ALT_MAX * -1;
}

/// <summary>
/// Resets altitude PID controller
/// </summary>
void pid_altitude_reset(void) {
	// Reset GPS PID controller
	alt_total_previous = actual_pressure;
	alt_total_avarage = 0;
	alt_rotating_mem_location = 0;

	// Reset the output of the PID controller
	pid_i_mem_alt = 0;
	pid_output_alt = 0;

	// Reset setpoint
	pid_alt_setpoint = actual_pressure;
}
