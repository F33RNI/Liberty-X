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
/// Calculates roll, pitch and yaw PID controllers
/// </summary>
void pid_roll_pitch_yaw(void) {
    // Reset setpoints
    pid_roll_setpoint = 0;
    pid_pitch_setpoint = 0;
    pid_yaw_setpoint = 0;

    // Add a little deadband of 16us for better results.
    if (pid_roll_setpoint_base > 1508)
        pid_roll_setpoint = pid_roll_setpoint_base - 1508;
    else if (pid_roll_setpoint_base < 1492)
        pid_roll_setpoint = pid_roll_setpoint_base - 1492;

    if (pid_pitch_setpoint_base > 1508)
        pid_pitch_setpoint = pid_pitch_setpoint_base - 1508;
    else if (pid_pitch_setpoint_base < 1492)
        pid_pitch_setpoint = pid_pitch_setpoint_base - 1492;

    if (pid_yaw_setpoint_base > 1508)
        pid_yaw_setpoint = pid_yaw_setpoint_base - 1508;
    else if (pid_yaw_setpoint_base < 1492)
        pid_yaw_setpoint = pid_yaw_setpoint_base - 1492;

    // Get angles in degrees.  max pitch rate is aprox (500-8)/3 = 164deg/s
    pid_roll_setpoint -= roll_level_adjust;
    pid_roll_setpoint /= 3.0;

    pid_pitch_setpoint -= pitch_level_adjust;
    pid_pitch_setpoint /= 3.0;

    pid_yaw_setpoint /= 3.0;

    // Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += PID_ROLL_I * pid_error_temp;
    if (pid_i_mem_roll > PID_ROLL_MAX)pid_i_mem_roll = PID_ROLL_MAX;
    else if (pid_i_mem_roll < PID_ROLL_MAX * -1)pid_i_mem_roll = PID_ROLL_MAX * -1;

    pid_output_roll = PID_ROLL_P * pid_error_temp + pid_i_mem_roll + PID_ROLL_D * (pid_error_temp - pid_last_roll_d_error);
    if (pid_output_roll > PID_ROLL_MAX)pid_output_roll = PID_ROLL_MAX;
    else if (pid_output_roll < PID_ROLL_MAX * -1)pid_output_roll = PID_ROLL_MAX * -1;

    pid_last_roll_d_error = pid_error_temp;

    // Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += PID_PITCH_I * pid_error_temp;
    if (pid_i_mem_pitch > PID_PITCH_MAX)pid_i_mem_pitch = PID_PITCH_MAX;
    else if (pid_i_mem_pitch < PID_PITCH_MAX * -1)pid_i_mem_pitch = PID_PITCH_MAX * -1;

    pid_output_pitch = PID_PITCH_P * pid_error_temp + pid_i_mem_pitch + PID_PITCH_D * (pid_error_temp - pid_last_pitch_d_error);
    if (pid_output_pitch > PID_PITCH_MAX)pid_output_pitch = PID_PITCH_MAX;
    else if (pid_output_pitch < PID_PITCH_MAX * -1)pid_output_pitch = PID_PITCH_MAX * -1;

    pid_last_pitch_d_error = pid_error_temp;

    // Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += PID_YAW_I * pid_error_temp;
    if (pid_i_mem_yaw > PID_YAW_MAX)pid_i_mem_yaw = PID_YAW_MAX;
    else if (pid_i_mem_yaw < PID_YAW_MAX * -1)pid_i_mem_yaw = PID_YAW_MAX * -1;

    pid_output_yaw = PID_YAW_P * pid_error_temp + pid_i_mem_yaw + PID_YAW_D * (pid_error_temp - pid_last_yaw_d_error);
    if (pid_output_yaw > PID_YAW_MAX)pid_output_yaw = PID_YAW_MAX;
    else if (pid_output_yaw < PID_YAW_MAX * -1)pid_output_yaw = PID_YAW_MAX * -1;

    pid_last_yaw_d_error = pid_error_temp;
}

/// <summary>
/// Resets the roll, pitch and yaw PID controllers
/// </summary>
void pid_roll_pitch_yaw_reset(void) {
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pid_output_roll = 0;
    pid_output_pitch = 0;
    pid_output_yaw = 0;
}
