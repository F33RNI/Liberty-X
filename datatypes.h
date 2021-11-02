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

#ifndef DATATYPES_H
#define DATATYPES_H

// Common variables
uint8_t start, flight_mode, error;
uint16_t count_var;
uint32_t loop_timer;

// Motors
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, takeoff_throttle;
float throttle_exp;

// Voltmeter
float battery_voltage;

// IMU
int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;
int32_t acc_roll_cal, acc_pitch_cal;
boolean acc_calibration_flag, gyro_calibration_flag;
int32_t acc_total_vector, acc_total_vector_at_start;

// Angles
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float roll_level_adjust, pitch_level_adjust;

// PID
float pid_error_temp;
float pid_output_roll, pid_output_pitch, pid_output_yaw;
int32_t channel_1_base, channel_2_base, pid_roll_setpoint_base, pid_pitch_setpoint_base, pid_yaw_setpoint_base;
float pid_i_mem_roll, pid_roll_setpoint, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_last_yaw_d_error;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

// GPS PID
int16_t pid_output_gps_lat, pid_output_gps_lon;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[35], gps_lon_rotating_mem[35];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;

// Vertical acceleration
int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total;
int16_t acc_z_average_short[25], acc_z_average_long[50];
uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
int32_t acc_alt_integrated;

// Barometer
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[5], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
int32_t pressure_rotating_mem[20], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
int32_t dT, dT_C5;

// Altitude hold PID
float pid_i_mem_alt, pid_alt_setpoint, pid_output_alt, pid_error_gain_altitude;
uint8_t alt_rotating_mem_location;
float alt_rotating_mem[30], alt_total_avarage;
float alt_total_previous;

// Compass
boolean compass_calibration_flag, heading_lock_enabled;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;

// GPS
uint8_t gps_buffer[20];
uint8_t gps_buffer_position, gps_byte_previous;
uint8_t gps_check_byte, gps_temp_byte;
uint8_t gps_lost_counter = UINT8_MAX;
uint8_t number_used_sats;
uint8_t hdop;
int16_t altitude;
uint16_t ground_heading;
uint16_t ground_speed;
uint8_t gps_setpoint_set;
boolean new_gps_data_available;
int32_t l_lat_gps, l_lon_gps, l_lat_setpoint, l_lon_setpoint;
float gps_pitch_adjust, gps_roll_adjust;
float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;

// LED
uint8_t leds_tick_counter;
boolean leds_red_state, leds_green_state;
uint8_t leds_error_counter, leds_flight_mode_counter;
uint16_t leds_loop_counter, leds_onboard_loop_counter, leds_error_loop_counter;
boolean buildin_led_state;

// Receiver
int32_t channel_1, channel_2, channel_3, channel_4, channel_5, channel_6, channel_7, channel_8;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;
boolean takeoff_detected;

// Gimbal
uint32_t gimbal_pitch;

// Liberty-Link
#ifdef LIBERTY_LINK
boolean link_allowed, link_telemetry_allowed, link_direct_control;
int16_t direct_roll_control = 1500, direct_pitch_control = 1500, direct_yaw_control = 1500, direct_throttle_control = 1500;
uint8_t link_system_byte, link_system_data, link_waypoint_step;
uint8_t link_buffer[12], link_buffer_counter, link_byte_previous;
uint8_t link_check_byte, link_temp_byte;
uint8_t link_lost_counter = UINT8_MAX;

int32_t waypoints_lat[16], waypoints_lon[16];
uint8_t waypoints_command[16];
uint8_t waypoints_index;
int32_t l_lat_waypoint, l_lon_waypoint, l_lat_waypoint_last, l_lon_waypoint_last;
float waypoint_lat_factor, waypoint_lon_factor, waypoint_move_factor;
#endif

// Telemetry
#ifdef TELEMETRY
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter, telemetry_check_byte, telemetry_burst_counter;
uint32_t telemetry_buffer_bytes;
#endif

// Sonars
#ifdef SONARUS
uint8_t sonarus_cycle_counter;
uint16_t sonar_1_raw, sonar_2_raw;
#ifdef SONARUS_TAKEOFF_DETECTION
uint16_t sonar_2_at_start, sonar_2_prev;
#endif
#ifdef LIBERTY_LINK
float pid_i_mem_sonar, pid_sonar_setpoint, pid_last_sonar_d_error, pid_output_sonar;
#endif

#endif

// Lux meter
#ifdef LUX_METER
uint8_t lux_cycle_counter;
uint16_t lux_raw_data;
float lux_data;
uint8_t lux_sqrt_data;
#endif

// Debugger
#ifdef DEBUGGER
uint16_t debugger_loop_counter;
#endif

#endif

