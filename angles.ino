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
/// Calculates input values for the pid controllers and angles
/// with the gyro and acc
/// </summary>
void calculate_angles(void) {
	// Gyro PID input. 65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information)
	gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);
	gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
	gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);

	// Gyro angle calculations. 0.0000611 = 1 / (250Hz / 65.5)
	angle_pitch += (float)gyro_pitch * 0.0000611;
	angle_roll += (float)gyro_roll * 0.0000611;
	angle_yaw += (float)gyro_yaw * 0.0000611;

	// Limit yaw angle
	if (angle_yaw < 0) angle_yaw += 360;
	else if (angle_yaw >= 360) angle_yaw -= 360;

	// If the IMU has yawed transfer the roll angle to the pitch angle
	angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.0000611 * DEG_TO_RAD);
	angle_roll += angle_pitch * sin((float)gyro_yaw * 0.0000611 * DEG_TO_RAD);

	// Calculate the difference between the gyro and compass heading and make a small correction
	angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 650.0;  // 780.0

	// Limit yaw angle
	if (angle_yaw < 0) angle_yaw += 360;
	else if (angle_yaw >= 360) angle_yaw -= 360;

	// Accelerometer angle calculations
	// Calculate the total accelerometer vector
	acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

	// Prevent the asin function to produce a NaN
	if (abs(acc_y) < acc_total_vector)
		angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
	if (abs(acc_x) < acc_total_vector)
		angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;

	// Correct the drift of the gyro pitch angle with the accelerometer angles (default = 0.9996, 0.0004)
	angle_pitch = angle_pitch * 0.9992 + angle_pitch_acc * 0.0008;
	angle_roll = angle_roll * 0.9992 + angle_roll_acc * 0.0008;

	// Calculate the angle corrections
	pitch_level_adjust = angle_pitch * 15;
	roll_level_adjust = angle_roll * 15;
}
