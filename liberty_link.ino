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

#ifdef LIBERTY_LINK

 /// <summary>
 /// Allows to control the drone directly or send altitude or GPS waypoint, for example from the landing platform
 /// For more please visit https://github.com/XxOinvizioNxX/Liberty-Way
 /// </summary>
void liberty_link(void) {
    // Count lost frames
    if (link_lost_counter < UINT8_MAX)
        link_lost_counter++;

    while (TELEMETRY_SERIAL.available()) {
        // Read current byte
        link_buffer[link_buffer_counter] = TELEMETRY_SERIAL.read();

        if (link_byte_previous == LINK_SUFFIX_1 && link_buffer[link_buffer_counter] == LINK_SUFFIX_2) {
            // If data suffix appears
            // Reset buffer position
            link_buffer_counter = 0;

            // Reset check sum
            link_check_byte = 0;

            // Calculate check sum
            for (link_temp_byte = 0; link_temp_byte <= 8; link_temp_byte++)
                link_check_byte ^= link_buffer[link_temp_byte];

            if (link_check_byte == link_buffer[9]) {
                // If the check sums are equal
                // Reset watchdog
                link_lost_counter = 0;

                // Send one byte of telemetry as answer
                link_telemetry_allowed = 1;

                // Reset direct corrections
                direct_roll_control = 1500;
                direct_pitch_control = 1500;
                direct_yaw_control = 1500;
                direct_throttle_control = 1500;

                // Parse data
                // Link command:
                // 0 - IDLE, 
                // 1 - Direct control, 
                // 2 - Pressure (altitude) setpoint, 
                // 3 - GPS setpoint, 
                // 4 - Turn off the motors, 
                // 5 - Start Liberty Way sequence, 
                // 6 - Abort Liberty Way
                link_command = (uint32_t)link_buffer[8];
                if (link_command == 1) {
                    // Direct control
                    direct_roll_control = (uint32_t)link_buffer[1] | (uint32_t)link_buffer[0] << 8;
                    direct_pitch_control = (uint32_t)link_buffer[3] | (uint32_t)link_buffer[2] << 8;
                    direct_yaw_control = (uint32_t)link_buffer[5] | (uint32_t)link_buffer[4] << 8;
                    direct_throttle_control = (uint32_t)link_buffer[7] | (uint32_t)link_buffer[6] << 8;

                    // Check data range
                    if (direct_roll_control < 1000 || direct_roll_control > 2000 ||
                        direct_pitch_control < 1000 || direct_pitch_control > 2000 ||
                        direct_yaw_control < 1000 || direct_yaw_control > 2000 ||
                        direct_throttle_control < 1000 || direct_throttle_control > 2000) {
                        // Disable any corrections and show error
                        link_command = 0;
                        error = 9;
                    }
                }
                else if (link_command == 2) {
                    // Pressure waypoint only after ascending
                    pressure_waypoint = (float)((int32_t)link_buffer[3] | (int32_t)link_buffer[2] << 8 | (int32_t)link_buffer[1] << 16 | (int32_t)link_buffer[0] << 24);

                    if (pressure_waypoint < 1000)
                        // Do not change pressure waypoint if less then 1000 Pa
                        pressure_waypoint = pid_alt_setpoint;

                    // Check data range and other packet bytes
                    if (pressure_waypoint > 120000 ||
                        link_buffer[4] || link_buffer[5] ||
                        link_buffer[6] || link_buffer[7]) {
                        // Disable any corrections and show error
                        pressure_waypoint = actual_pressure;
                        link_command = 0;
                        error = 9;
                    }
                    else
                        // Set flag if all checks passed
                        link_new_waypoint_altitude = 1;
                }
                else if (link_command == 3) {
                    // GPS waypoint
                    l_lat_waypoint = (int32_t)link_buffer[3] | (int32_t)link_buffer[2] << 8 | (int32_t)link_buffer[1] << 16 | (int32_t)link_buffer[0] << 24;
                    l_lon_waypoint = (int32_t)link_buffer[7] | (int32_t)link_buffer[6] << 8 | (int32_t)link_buffer[5] << 16 | (int32_t)link_buffer[4] << 24;

                    // Check data range
                    if (l_lat_waypoint <= -90000000 || l_lat_waypoint >= 90000000 ||
                        l_lon_waypoint <= -180000000 || l_lon_waypoint >= 18000000) {
                        // Disable any corrections and show error
                        l_lat_waypoint = l_lat_gps;
                        l_lon_waypoint = l_lon_gps;
                        link_command = 0;
                        error = 9;
                    }
                    else {
                        // Set flag if all checks passed
                        if (link_waypoint_step == 6)
                            // If liberty-link is in the setpoint mode
                            link_new_waypoint_gps = 1;
                        else if (abs(l_lat_setpoint - l_lat_waypoint) > GPS_SETPOINT_MAX_DISTANCE * 2
                            || abs(l_lon_setpoint - l_lon_waypoint) > GPS_SETPOINT_MAX_DISTANCE * 2)
                            // If liberty-link is not in the setpoint mode and it's a realy new position
                            link_new_waypoint_gps = 1;
                    }
                }
                else if (link_command == 4 || link_command == 5) {
                    // 4 - Turn off the motors
                    // 5 - Start Liberty Way sequence
                    // Check other packet bytes
                    if (link_buffer[0] || link_buffer[1] ||
                        link_buffer[2] || link_buffer[3] ||
                        link_buffer[4] || link_buffer[5] ||
                        link_buffer[6] || link_buffer[7]) {
                        // Disable any corrections and show error
                        link_command = 0;
                        error = 9;
                    }
                }
                else if (link_command > 0 && link_command != 6) {
                    // 6 - Abort Liberty Way
                    // Unknown command
                    link_command = 0;
                    error = 9;
                }
            }
            else
                link_lost_counter = UINT8_MAX;
        }
        else {
            // Store data bytes
            link_byte_previous = link_buffer[link_buffer_counter];
            link_buffer_counter++;

            // Reset buffer on overflow
            if (link_buffer_counter > 11)link_buffer_counter = 0;
        }
    }

    if (link_allowed) {
        // Default flight mode for all Liberty Way sequences
        flight_mode = 3;

        if (link_lost_counter < LINK_LOST_CYCLES) {
            if (link_command == 1) {
                // Flight mode for direct control
                flight_mode = 1;
            }
            else if (link_command == 4) {
#ifdef SONARUS
                if (sonar_2_raw && sonar_2_raw < SONARUS_LINK_MTOF) {
                    // Turn off the motors if altitude is less than SONARUS_LINK_MTOF and not equal to 0
                    start = 0;
                    takeoff_detected = 0;
                    // Reset GPS and altitude flight variables
                    link_waypoint_step = 0;
                    link_new_waypoint_altitude = 0;
                    link_new_waypoint_gps = 0;
                    // Reset link command
                    link_command = 0;
                }
#else
                // Turn off the motors
                start = 0;
                takeoff_detected = 0;
                // Reset GPS and altitude flight variables
                link_waypoint_step = 0;
                link_new_waypoint_altitude = 0;
                link_new_waypoint_gps = 0;
                // Reset link command
                link_command = 0;
#endif
            }

            // Start Liberty Way sequence if link allowed, working and all waypoints provided
            if (link_new_waypoint_altitude && link_new_waypoint_gps && number_used_sats >= LINK_MIN_NUM_SATS) {
                if (link_waypoint_step == 0 && link_command == 5) {
                    // Begin Liberty Way sequence
                    link_waypoint_step = 1;

                    // Reset new waypoints flags
                    link_new_waypoint_altitude = 0;
                    link_new_waypoint_gps = 0;
                }
                else if (link_waypoint_step > 3) {
                    // Recalculate correction for new position if GPS available
                    link_waypoint_step = 4;

                    // Reset new waypoints flags
                    link_new_waypoint_altitude = 0;
                    link_new_waypoint_gps = 0;
                }
                if (link_command > 0 && link_command != 6)
                    // Reset link_aborted on new data
                    link_aborted = 0;
            }
        }
        else {
            // Liberty Way lost
            // Abort Liberty-Way
            if (link_command == 1 && !link_aborted)
                liberty_link_abort();

            // Reset link command
            link_command = 0;
        }
    }

    if (!link_aborted && (/*error > 1 || number_used_sats < LINK_MIN_NUM_SATS || */ !link_allowed || link_command == 6)) {
        // Abort Liberty Way if not allowed or an error occurs
        liberty_link_abort();
    }

    if (!link_aborted) {
        // Liberty-Way WAYP sequence only if not aborted

        // Step 1. Take off
        if (link_waypoint_step == 1) {
            if (/*!error &&*/ start < 2 && channel_3 > 1050 && channel_6 > 1500) {
                ground_pressure = actual_pressure;
                start = 1;
                link_waypoint_step = 2;
            }
            else if (takeoff_detected)
                // Go to step altitude waypoint calculations if takeoff detected
                link_waypoint_step = 3;
        }
        // Step 2. Waiting for takeoff
        else if (link_waypoint_step == 2) {
            if (takeoff_detected)
                // Go to step altitude waypoint calculations if takeoff detected
                link_waypoint_step = 3;
        }
        // Step 3. Calculate the distance and reduse the pressure setpoint and increase the altitude
        else if (link_waypoint_step == 3) {
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE) {
                // If the drone is nearby, slightly increase the altitude
                if (pid_alt_setpoint - (ground_pressure - NEAR_PRESSURE_ASCEND) > 1.0)
                    // Decrease pressure (increase altitude) until waypoint is reached
                    pid_alt_setpoint -= WAYPOINT_ALTITUDE_TERM;
                else
                    // Go to gps pre-calculations as soon as the waypoint is reached
                    link_waypoint_step = 4;
            }
            else {
                // If the drone is far away, increase the altitude significantly (so as not to crash into buildings)
                if (pid_alt_setpoint - (ground_pressure - FAR_PRESSURE_ASCEND) > 1.0)
                    // Decrease pressure (increase altitude) until waypoint is reached
                    pid_alt_setpoint -= WAYPOINT_ALTITUDE_TERM;
                else
                    // Go to gps pre-calculations as soon as the waypoint is reached
                    link_waypoint_step = 4;
            }
        }
        // Step 4. Calculate the distance and factors to the GPS waypoint
        else if (link_waypoint_step == 4) {
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                // If the drone is nearby, go to the GPS stabilization and altitude decreasing
                link_waypoint_step = 6;
            else {
                // Reset adjustments if needed
                if ((l_lat_waypoint > l_lat_setpoint && l_lat_waypoint_last < l_lat_setpoint) ||
                    (l_lat_waypoint < l_lat_setpoint && l_lat_waypoint_last > l_lat_setpoint))
                    // Reset latitude float adjustments because direction has changed
                    l_lat_gps_float_adjust = 0;
                if ((l_lon_waypoint > l_lon_setpoint && l_lon_waypoint_last < l_lon_setpoint) ||
                    (l_lon_waypoint < l_lat_setpoint && l_lon_waypoint_last > l_lon_setpoint))
                    // Reset latitude float adjustments because direction has changed
                    l_lon_gps_float_adjust = 0;

                if (l_lat_waypoint != l_lat_waypoint_last || l_lon_waypoint != l_lon_waypoint_last) {
                    // Reset factors because the need to be recalculated
                    waypoint_lat_factor = 0;
                    waypoint_lon_factor = 0;

                    // Calculate factors
                    if (abs(l_lat_waypoint - l_lat_setpoint) >= abs(l_lon_waypoint - l_lon_setpoint)) {
                        waypoint_lon_factor = (float)abs(l_lon_waypoint - l_lon_setpoint) / (float)abs(l_lat_waypoint - l_lat_setpoint);
                        waypoint_lat_factor = 1;
                    }
                    else {
                        waypoint_lon_factor = 1;
                        waypoint_lat_factor = (float)abs(l_lat_waypoint - l_lat_setpoint) / (float)abs(l_lon_waypoint - l_lon_setpoint);
                    }
                }

                // Go to GPS waypoint flight
                link_waypoint_step = 5;

                // Store new waypoints
                l_lat_waypoint_last = l_lat_waypoint;
                l_lon_waypoint_last = l_lon_waypoint;
            }
        }
        // Step 5. GPS waypoint flight
        else if (link_waypoint_step == 5) {
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                // If the drone is nearby, go to the GPS stabilization
                link_waypoint_step = 6;

            // Calculate speed factor
            if (abs(l_lat_waypoint - l_lat_setpoint) < 160 && abs(l_lon_waypoint - l_lon_setpoint) < 160 && waypoint_move_factor > WAYPOINT_GPS_MIN_FACTOR)
                waypoint_move_factor -= 0.00015;
            else if (waypoint_move_factor < WAYPOINT_GPS_MAX_FACTOR)
                waypoint_move_factor += 0.0001;

            // Calculate adjustments
            if (l_lat_waypoint != l_lat_setpoint) {
                if (l_lat_waypoint > l_lat_setpoint) l_lat_gps_float_adjust += waypoint_move_factor * waypoint_lat_factor;
                if (l_lat_waypoint < l_lat_setpoint) l_lat_gps_float_adjust -= waypoint_move_factor * waypoint_lat_factor;
            }
            if (l_lon_waypoint != l_lon_setpoint) {
                if (l_lon_waypoint > l_lon_setpoint) l_lon_gps_float_adjust += waypoint_move_factor * waypoint_lon_factor;
                if (l_lon_waypoint < l_lon_setpoint) l_lon_gps_float_adjust -= waypoint_move_factor * waypoint_lon_factor;
            }
        }
        // Step 6. GPS setpoint stabilization (direct control of setpoints) and increasing the pressure (decreasing the altitude)
        else if (link_waypoint_step == 6) {
            // Set GPS setpoint
            l_lat_setpoint = l_lat_waypoint;
            l_lon_setpoint = l_lon_waypoint;

            if (link_lost_counter < LINK_LOST_CYCLES) {
                // Check if Liberty Link available before descending
                // Pressure waypoint
                if (pid_alt_setpoint < pressure_waypoint)
                    // Increase pressure (decrease altitude)
                    pid_alt_setpoint += WAYPOINT_ALTITUDE_TERM;
                else if (pid_alt_setpoint > pressure_waypoint)
                    // Decrease pressure (increase altitude)
                    pid_alt_setpoint -= WAYPOINT_ALTITUDE_TERM;
            }
        }
    }
}

/// <summary>
/// Clears flags, resets direct corrections, waypoint flags and sharply jumps up to prevent a collision
/// </summary>
void liberty_link_abort(void) {
    // Reset GPS and altitude flight variables
    link_new_waypoint_altitude = 0;
    link_new_waypoint_gps = 0;

    // Reset direct corrections
    direct_roll_control = 1500;
    direct_pitch_control = 1500;
    direct_yaw_control = 1500;
    direct_throttle_control = 1500;

    // Fly up sharply to prevent a collision
    pid_alt_setpoint = actual_pressure - ABORT_PRESSURE_ASCEND;

    // Set current GPS position as setpoint
    l_lat_setpoint = l_lat_gps;
    l_lon_setpoint = l_lon_gps;
    l_lat_waypoint = l_lat_gps;
    l_lon_waypoint = l_lon_gps;

    // Reset GPS corrections
    l_lat_gps_float_adjust = 0;
    l_lon_gps_float_adjust = 0;
    waypoint_move_factor = 0;
    pid_gps_reset();

    // Set aborted flag
    link_aborted = 1;
}

#endif
