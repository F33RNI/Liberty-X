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

#ifdef LIBERTY_LINK

 /// <summary>
 /// Allows to control the drone from the landing platform
 /// For more please visit https://github.com/XxOinvizioNxX/Liberty-Way
 /// </summary>
void liberty_link_handler(void) {
    // Clear direct_control flag if LibertyLink is lost
    if (!link_allowed || link_lost_counter >= LEDS_BLINK_CYCLES)
        link_direct_control = 0;

    if (link_allowed) {
        // Default flight mode for all Liberty Way sequences
        flight_mode = 3;

        // Change flight mode to 1 in direct control
        if (link_direct_control)
            flight_mode = 1;


        // ---------------------------------------------
        // Step 1. Waiting for takeoff
        // ---------------------------------------------
        if (link_waypoint_step == 1) {
            // Go to step 2 if takeoff detected
            if (takeoff_detected)
                link_waypoint_step = 2;
        }

        // ---------------------------------------------
        // Step 2. Reduse the pressure setpoint to increase the altitude
        // ---------------------------------------------
        else if (link_waypoint_step == 2) {
            // Go to step 3 as soon as the waypoint is reached
            if (pid_alt_setpoint < ground_pressure - LINK_PRESSURE_ASCEND)
                link_waypoint_step = 3;

            // Decrease pressure (increase altitude) until waypoint is reached
            else
                pid_alt_setpoint -= WAYPOINT_ALTITUDE_TERM;
        }

        // ---------------------------------------------
        // Step 3. Calculate next waypoint
        // ---------------------------------------------
        else if (link_waypoint_step == 3) {
            // Check if the waypoint is available
            if (waypoints_command[waypoints_index] > 0) {
                // Select waypoint from array of waypoints
                l_lat_waypoint = waypoints_lat[waypoints_index];
                l_lon_waypoint = waypoints_lon[waypoints_index];

                // If the drone is nearby to the waypoint, go to the GPS setpoint
                if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                    && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                    link_waypoint_step = 6;

                // Go to step 4 if the drone if far from the waypoint
                else
                    link_waypoint_step = 4;
            }

            // Switch to next point or start automatic landing if waypoint is not available
            else {
                // Incrememnt the waypoint index counter until the entire array has been scanned
                if (waypoints_index < 15)
                    waypoints_index++;

                // Start auto landing if there are no waypoints
                else
                    link_waypoint_step = 7;
            }
        }

        // ---------------------------------------------
        // Step 4. Calculate the distance and factors to the GPS waypoint
        // ---------------------------------------------
        else if (link_waypoint_step == 4) {
            // If the drone is nearby to the waypoint, go to the GPS setpoint
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                link_waypoint_step = 6;

            // If the drone if far from the waypoint
            else {
                // Reset latitude float adjustments if direction has changed
                if ((l_lat_waypoint > l_lat_setpoint && l_lat_waypoint_last < l_lat_setpoint) ||
                    (l_lat_waypoint < l_lat_setpoint && l_lat_waypoint_last > l_lat_setpoint))
                    l_lat_gps_float_adjust = 0;

                // Reset latitude float adjustments if direction has changed
                if ((l_lon_waypoint > l_lon_setpoint && l_lon_waypoint_last < l_lon_setpoint) ||
                    (l_lon_waypoint < l_lat_setpoint && l_lon_waypoint_last > l_lon_setpoint))
                    l_lon_gps_float_adjust = 0;

                // Recalculate factors if waypoint has changed
                if (l_lat_waypoint != l_lat_waypoint_last || l_lon_waypoint != l_lon_waypoint_last) {
                    // Reset factors because they need to be recalculated
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

        // ---------------------------------------------
        // Step 5. GPS waypoint flight
        // ---------------------------------------------
        else if (link_waypoint_step == 5) {
            // If the drone is nearby to the waypoint, go to the GPS setpoint
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                link_waypoint_step = 6;

            // Calculate speed factor
            if (abs(l_lat_waypoint - l_lat_setpoint) < 160 && abs(l_lon_waypoint - l_lon_setpoint) < 160
                && waypoint_move_factor > WAYPOINT_GPS_MIN_FACTOR)
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

        // ---------------------------------------------
        // Step 6. GPS setpoint aka GPS direct control
        // ---------------------------------------------
        else if (link_waypoint_step == 6) {
            // Set GPS setpoint
            l_lat_setpoint = l_lat_waypoint;
            l_lon_setpoint = l_lon_waypoint;

            // Reset setpoint of sonarus
            pid_sonar_setpoint = 0;

            // Start auto landing if waypoint command is 111 (7)
            if (waypoints_command[waypoints_index] == 0b111)
                link_waypoint_step = 7;

            // Switch to next waypoint if waypoint command is 100 (4)
            else if (waypoints_command[waypoints_index] == 0b100 && waypoints_index < 15) {
                waypoints_index++;
                link_waypoint_step = 3;
            }

            // Set sonarus setpoint to 4 meters if waypoint command is 010 (2) or 101 (5)
            else if (waypoints_command[waypoints_index] == 0b010 || waypoints_command[waypoints_index] == 0b101)
                pid_sonar_setpoint = 4000;

            // Set sonarus setpoint to 2 meters if waypoint command is 011 (3) or 110 (6)
            // TODO: Change 1000 back to 2000. (the value 1000mm was set for debugging purposes)
            else if (waypoints_command[waypoints_index] == 0b011 || waypoints_command[waypoints_index] == 0b110)
                pid_sonar_setpoint = 1000;

            // If a descending is required
            if (pid_sonar_setpoint != 0) {
#ifdef SONARUS
                // If sonarus if working and the current height is not higher than half a meter above the setpoint
                if (sonar_2_raw > 0 && (float)sonar_2_raw - pid_sonar_setpoint < 500) {
                    // Disable pressure control
                    pid_alt_setpoint = actual_pressure;

                    // Execute Sonarus PID controller
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

                // Increase pressure (decrease altitude) and reset sonarus PID if sonarus cannot see the ground
                else {
                    pid_alt_setpoint += WAYPOINT_ALTITUDE_TERM;
                    pid_output_sonar = 0;
                    pid_i_mem_sonar = 0;
                    pid_last_sonar_d_error = 0;
                }
#else
                // Increase pressure (decrease altitude) if there is no sonarus module
                pid_alt_setpoint += WAYPOINT_ALTITUDE_TERM;
#endif
            }
        }

        // ---------------------------------------------
        // Step 7. Altitude reduction for auto-landing
        // ---------------------------------------------
        else if (link_waypoint_step == 7) {
            // Disable motors if current altitude stops decreasing
            if (pid_alt_setpoint > actual_pressure + 150)
                link_waypoint_step = 8;

            // Increase pressure (decrease altitude)
            pid_alt_setpoint += WAYPOINT_ALTITUDE_TERM;
        }

        // ---------------------------------------------
        // Step 8. Turn off the motors
        // ---------------------------------------------
        else if (link_waypoint_step == 8) {
            // Turn off the motors
            link_check_and_turnoff_motors();

            // Reset link_waypoint_step
            if (start == 0)
                link_waypoint_step = 0;
        }
    }
}

/// <summary>
/// Performs pre-flight checks, auto take-off and begins the Liberty-Way sequence
/// </summary>
void link_start_and_takeoff(void) {
    // Perform pre-flight checks
    if (start < 1 && channel_3 > 1050 && channel_6 > 1500
        && number_used_sats >= LINK_MIN_NUM_SATS && battery_voltage >= LINK_MIN_BAT_VOLTAGE) {

        // Remember ground pressure
        ground_pressure = actual_pressure;
        start = 1;

        // Start Liberty-Way sequence
        link_waypoint_step = 1;

        // Start from beggining of the waypoints array
        waypoints_index = 0;
    }
}

/// <summary>
/// Clears array of waypoints and some other variables 
/// when the motors are turning off
/// </summary>
void link_clear_disarm(void) {
    // Clear arrays of waypoints
    for (count_var = 0; count_var < 16; count_var++) {
        waypoints_lat[count_var] = 0;
        waypoints_lon[count_var] = 0;
        waypoints_command[count_var] = 0;
    }

    // Clear waypoint step
    link_waypoint_step = 0;

    // Clear direct control flag
    link_direct_control = 0;

    // Clear current waypoint index
    waypoints_index = 0;
}

/// <summary>
/// Turns off the motors only if the drone is below a certain height (if SONARUS module is enabled)
/// </summary>
void link_check_and_turnoff_motors(void) {
    if (start > 0) {
        // Turn off the motors if SONARUS is enabled and altitude is less than SONARUS_LINK_MTOF and not equal to 0
#if (defined(SONARUS) && defined(SONARUS_MTOF_PROTECTION))
        if (sonar_2_raw && sonar_2_raw < SONARUS_LINK_MTOF) {
            start = 0;
            takeoff_detected = 0;
            link_clear_disarm();
        }

        // Directly turn off the motors if SONARUS is disabled
#else
        start = 0;
        takeoff_detected = 0;
        link_clear_disarm();
#endif
    }
}

/// <summary>
/// Resets direct corrections, waypoint flags and sharply jumps up to prevent a collision
/// </summary>
void direct_control_abort(void) {
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
}

/// <summary>
/// Flight Termination System of Liberty-X
/// In the event of an error, to prevent victims, turns off the motors :(
/// </summary>
void liberty_x_fts(void) {
    // Enter infinite loop
    while (true)
    {
        // Set error to 8 (FTS)
        error = 8;

        // Blink with LEDs
        leds_error_signal();

        // Disable the motors (start beeping)
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
}

#endif
