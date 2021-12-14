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

// TODO: ADD AUTO-HEADING DURING WAYPOINTS FLY

#ifdef LIBERTY_LINK

 /// <summary>
 /// Allows to control the drone from the landing platform
 /// For more please visit https://github.com/XxOinvizioNxX/Liberty-Way
 /// </summary>
void liberty_link_handler(void) {
    // Clear direct_control flag if LibertyLink is lost
    if (!link_allowed || link_lost_counter >= LINK_LOST_CYCLES)
        link_direct_control = 0;

    if (link_allowed && !auto_landing_step) {
        // Increment loop counter (timer)
        if (link_waypoint_loop_counter < UINT16_MAX)
            link_waypoint_loop_counter++;

        // Default flight mode for all Liberty Way sequences
        flight_mode = 3;

        // Change flight mode to 1 in direct control
        if (link_direct_control)
            flight_mode = 1;

        // Allpy yaw correction only if the drone is far from the waypoint
        if (abs(l_lat_setpoint - l_lat_waypoint) > GPS_SETPOINT_MAX_DISTANCE
            || abs(l_lon_setpoint - l_lon_waypoint) > GPS_SETPOINT_MAX_DISTANCE) {
            // Course P controller
            if (abs(waypoint_course - angle_yaw) > 180)
                waypoint_yaw_correction = (angle_yaw - waypoint_course) * WAYP_YAW_CORRECTION_TERM;
            else
                waypoint_yaw_correction = (waypoint_course - angle_yaw) * WAYP_YAW_CORRECTION_TERM;

            // Trim yaw correction
            if (waypoint_yaw_correction > WAYP_YAW_CORRECTION_MAX)
                waypoint_yaw_correction = WAYP_YAW_CORRECTION_MAX;
            else if (waypoint_yaw_correction < WAYP_YAW_CORRECTION_MAX * -1)
                waypoint_yaw_correction = WAYP_YAW_CORRECTION_MAX * -1;
        }

        // Reset yaw correction if the drone is near the waypoint
        else
            waypoint_yaw_correction = 0;

        // ---------------------------------------------
        // Step TAKEOFF. Waiting for takeoff
        // ---------------------------------------------
        if (link_waypoint_step == LINK_STEP_TAKEOFF) {
            // Go to ascending if takeoff detected
            if (takeoff_detected)
                link_waypoint_step = LINK_STEP_ASCENT;
        }

        // ---------------------------------------------
        // Step ASCENT. Reduse the pressure setpoint to increase the altitude
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_ASCENT) {
            // Go to step WAYP_CALC as soon as the pressure waypoint is reached
            if (pid_alt_setpoint <= ground_pressure - LINK_PRESSURE_ASCEND && actual_pressure <= pid_alt_setpoint + 10)
                link_waypoint_step = LINK_STEP_WAYP_CALC;

            // Decrease pressure (increase altitude) until waypoint is reached
            else if (pid_alt_setpoint >= ground_pressure - LINK_PRESSURE_ASCEND)
                pid_alt_setpoint -= WAYPOINT_ALTITUDE_TERM;
        }

        // ---------------------------------------------
        // Step WAYP_CALC. Calculate next waypoint
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_WAYP_CALC) {
            // Store current waypoint command
            waypoint_command = waypoints_command[waypoints_index];

            // Check if the waypoint is available
            if (waypoint_command > 0) {
                // Set the current setpoint as a waypoint if current command is DDC without GPS and descending
                if (waypoint_command == WAYP_CMD_BITS_DDC_NO_GPS_NO_DSC) {
                    l_lat_waypoint = l_lat_setpoint;
                    l_lon_waypoint = l_lon_setpoint;
                }

                // Select waypoint from array of waypoints
                else {
                    l_lat_waypoint = waypoints_lat[waypoints_index];
                    l_lon_waypoint = waypoints_lon[waypoints_index];
                }

                // Calculate course
                waypoint_course = atan2(l_lon_waypoint - l_lon_gps, l_lat_waypoint - l_lat_gps) * RAD_TO_DEG;
                if (waypoint_course < 0)
                    waypoint_course += 360;

                // If the drone is nearby to the waypoint, go to the GPS setpoint
                if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                    && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                    link_waypoint_step = LINK_STEP_GPS_SETP;

                // Calculate the distance and factors to the GPS waypoint
                else {
                    // If the drone is nearby to the waypoint, go to the GPS setpoint
                    if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                        && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                        link_waypoint_step = LINK_STEP_GPS_SETP;

                    // If the drone if far from the waypoint
                    else {
                        // Reset latitude float adjustments if direction has changed
                        if ((l_lat_waypoint > l_lat_setpoint && l_lat_waypoint_last < l_lat_setpoint) ||
                            (l_lat_waypoint < l_lat_setpoint && l_lat_waypoint_last > l_lat_setpoint))
                            l_lat_gps_float_adjust = 0;

                        // Reset latitude float adjustments if direction has changed
                        if ((l_lon_waypoint > l_lon_setpoint && l_lon_waypoint_last < l_lon_setpoint) ||
                            (l_lon_waypoint < l_lon_setpoint && l_lon_waypoint_last > l_lon_setpoint))
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
                        link_waypoint_step = LINK_STEP_GPS_WAYP;

                        // Store new waypoints
                        l_lat_waypoint_last = l_lat_waypoint;
                        l_lon_waypoint_last = l_lon_waypoint;
                    }
                }
            }

            // If waypoint is not available
            else {
                // Incrememnt the waypoint index counter until the entire array has been scanned
                if (waypoints_index < 15)
                    waypoints_index++;

                // Set the current setpoint as a waypoint if there are no more points available
                else {
                    l_lat_waypoint = l_lat_setpoint;
                    l_lon_waypoint = l_lon_setpoint;
                }

            }
        }

        // ---------------------------------------------
        // Step GPS_WAYP. GPS waypoint flight
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_GPS_WAYP) {
            // If the drone is nearby to the waypoint, go to the GPS setpoint
            if (abs(l_lat_setpoint - l_lat_waypoint) < GPS_SETPOINT_MAX_DISTANCE
                && abs(l_lon_setpoint - l_lon_waypoint) < GPS_SETPOINT_MAX_DISTANCE)
                link_waypoint_step = LINK_STEP_GPS_SETP;

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
        // Step GPS_SETP. GPS setpoint aka GPS direct control
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_GPS_SETP) {
            // Set GPS setpoint
            l_lat_setpoint = l_lat_waypoint;
            l_lon_setpoint = l_lon_waypoint;

            // Reset setpoint of sonarus
            pid_sonarus_setpoint = 0;

            // Start auto-landing sequence if waypoint command is landing
            if (waypoint_command == WAYP_CMD_BITS_LAND) {
                if (!auto_landing_step)
                    auto_landing_step = 1;
            }

            // Switch to next waypoint if waypoint command is just fly
            else if (waypoint_command == WAYP_CMD_BITS_FLY) {
                if (waypoints_index < 15) {
                    // Increment waypoint
                    waypoints_index++;

                    // Go to waypoint calculation
                    link_waypoint_step = LINK_STEP_WAYP_CALC;
                }
            }

            // Switch to parsel drop or descending if waypoint command is drop a parcel or descending
            else if (waypoint_command == WAYP_CMD_BITS_PARCEL || waypoint_command == WAYP_CMD_BITS_DESCEND) {
                link_waypoint_step = LINK_STEP_DESCENT;
            }

            // Set sonarus setpoint to SONARUS_DESCENT_MM if waypoint command is descending
            else if (waypoint_command == WAYP_CMD_BITS_DDC) {
                //link_waypoint_step = LINK_STEP_DESCENT;
            }
        }

        // ---------------------------------------------
        // Step DESCENT. Decrease the altitude
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_DESCENT) {
            // Switch to sonarus stabilization if the required height is reached
            if (sonarus_bottom > 0 && sonarus_bottom < SONARUS_DESCENT_MM) {
                link_waypoint_step = LINK_STEP_SONARUS;
                link_waypoint_loop_counter = 0;
            }
            
            else {
                // Increase pressure (decrease altitude)
                pid_alt_setpoint += WAYPOINT_ALTITUDE_TERM;

                // Reset sonarus PID controller
                sonarus_pid_reset();
            }
        }

        // ---------------------------------------------
        // Step SONARUS. Sonarus stabilization
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_SONARUS) {
            // Switch to dropping the parcel and next waypoint if not in direct control
            // and after waiting 1000 * 4ms = 4s to stabilize
            if (waypoint_command >= WAYP_CMD_BITS_FLY && link_waypoint_loop_counter >= 1000) {
                link_waypoint_step = LINK_STEP_AFTER_SONARUS;
                link_waypoint_loop_counter = 0;

                // Drop the parsel if current command is BITS_PARCEL
                if (waypoint_command == WAYP_CMD_BITS_PARCEL)
                    gimbal_pitch = 1000;
            }

            // Execute 2nd sonar PID controller
            pid_sonarus_setpoint = SONARUS_DESCENT_MM;
            sonarus_pid();
        }

        // ---------------------------------------------
        // Step AFTER_SONARUS. Waiting for a parcel to be dropped
        // ---------------------------------------------
        else if (link_waypoint_step == LINK_STEP_AFTER_SONARUS) {
            // Switch to altitude incresing after waiting 500 * 4ms = 2s
            if (link_waypoint_loop_counter >= 500) {
                // Incrememnt waypoint index
                if (waypoints_index < 15)
                    waypoints_index++;

                // Store new ground pressure
                if (sonarus_bottom > 0)
                    ground_pressure = actual_pressure + (float)sonarus_bottom / 84.2f;
                else
                    ground_pressure = actual_pressure + (float)SONARUS_DESCENT_MM / 84.2f;

                // Switch to altitude incresing
                link_waypoint_step = LINK_STEP_ASCENT;

                // Reset dropping servo if current command is BITS_PARCEL
                if (waypoint_command == WAYP_CMD_BITS_PARCEL)
                    gimbal_pitch = 2000;
            }

            // Execute 2nd sonar PID controller
            pid_sonarus_setpoint = SONARUS_DESCENT_MM;
            sonarus_pid();
        }
    }
}

/// <summary>
/// Performs auto take-off and begins the Liberty-Way sequence
/// </summary>
void link_start_and_takeoff(void) {
    if (link_allowed && !takeoff_detected && start < 1 && channel_3 > 1050 && channel_6 > 1500)
        link_takeoff_flag = 1;
}

/// <summary>
/// Starts Liberty-way sequence after takeoff
/// </summary>
void link_begin_sequence(void) {
    // Start Liberty-Way sequence
    link_waypoint_step = LINK_STEP_TAKEOFF;

    // Start from beggining of the waypoints array
    waypoints_index = 0;

    // Reset takeoff flag
    link_takeoff_flag = 0;
}

/// <summary>
/// Clears array of waypoints and some other variables 
/// when the motors are turning off
/// </summary>
void link_clear_disarm(void) {
    // Clear waypoint step
    link_waypoint_step = LINK_STEP_IDLE;

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
        if (!sonarus_bottom || sonarus_bottom < SONARUS_LINK_MTOF) {
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
        if (!error)
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
