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
/// Combines all corrections of roll, pitch and yaw axes for the PID controller
/// </summary>
void channel_collector(void) {
    // Initial values for mixing
    channel_1_base = channel_1;
    channel_2_base = channel_2;
    pid_yaw_setpoint_base = channel_4;
    gps_man_adjust_heading = angle_yaw;

    if (heading_lock_enabled) {
        // Heading lock
        heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
        channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * DEG_TO_RAD))
            + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * DEG_TO_RAD));
        channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * DEG_TO_RAD))
            + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * DEG_TO_RAD));
        gps_man_adjust_heading = course_lock_heading;
    }

    // Mix with the GPS if setpoint is set and the flight_mode >= 3
    if (flight_mode >= 3 && gps_setpoint_set) {
        pid_roll_setpoint_base = 1500 + gps_roll_adjust;
        pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
    }
    else {
        pid_roll_setpoint_base = channel_1_base;
        pid_pitch_setpoint_base = channel_2_base;
    }

#ifdef LIBERTY_LINK
    if (link_direct_control) {
        // Add direct roll/pitch/yaw control if direct control mode is enabled
        pid_roll_setpoint_base += direct_roll_control - 1500;
        pid_pitch_setpoint_base += direct_pitch_control - 1500;
        pid_yaw_setpoint_base += direct_yaw_control - 1500;
    }

    // Add waypoint yaw correction
    if (link_allowed && !auto_landing_step && 
        (link_waypoint_step == LINK_STEP_GPS_WAYP
            || link_waypoint_step == LINK_STEP_GPS_SETP 
            || link_waypoint_step == LINK_STEP_DESCENT))
        pid_yaw_setpoint_base += waypoint_yaw_correction;
#endif

#ifdef SONARUS
    // Anti-collision function
    if (sonar_1_raw > 0) {
        if (sonar_1_raw < SONARUS_SPRING_STOP)
            pid_pitch_setpoint_base += SONARUS_MAX_PITCH;
        else if (sonar_1_raw < SONARUS_SPRING_START)
            pid_pitch_setpoint_base += map(sonar_1_raw, SONARUS_SPRING_START, SONARUS_SPRING_STOP, 0, SONARUS_MAX_PITCH);
    }
#endif

    // Clip values
    if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
    if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
    if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
    if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;
    if (pid_yaw_setpoint_base > 2000)pid_yaw_setpoint_base = 2000;
    if (pid_yaw_setpoint_base < 1000)pid_yaw_setpoint_base = 1000;
}
