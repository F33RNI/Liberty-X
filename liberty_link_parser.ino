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
void liberty_link_parser(void) {
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
                // System byte:
                // 8 bits: PCCC XXXX
                // P - pointer (0 - waypoint, 1 - command)
                // CCC - command (+ XXXX in command mode)
                // XXXX - waypoint index (command bits in command mode)
                link_system_byte = link_buffer[8];

                // Parse XXXX (waypoint index or command data)
                link_system_data = link_system_byte & 0b00001111;

                // Parce CCC
                link_system_byte = (link_system_byte >> 4) & 0b00000111;

                // If link_system_byte >= 128 -> P=1 -> command mode
                if (link_system_byte >= 0b10000000) {

                    // CCC = 001 (1) -> Direct control
                    if (link_system_byte == 0b001
                        && waypoints_command[waypoints_index] > 0 && waypoints_command[waypoints_index] < 0b100) {
                        // Parse roll, pitch, yaw and throttle
                        direct_roll_control = (uint32_t)link_buffer[1] | (uint32_t)link_buffer[0] << 8;
                        direct_pitch_control = (uint32_t)link_buffer[3] | (uint32_t)link_buffer[2] << 8;
                        direct_yaw_control = (uint32_t)link_buffer[5] | (uint32_t)link_buffer[4] << 8;
                        direct_throttle_control = (uint32_t)link_buffer[7] | (uint32_t)link_buffer[6] << 8;

                        // Set direct control flag
                        link_direct_control = 1;
                    }

                    // CCC = 010 (2) -> Auto-takeoff
                    else if (link_system_byte == 0b010)
                        link_start_and_takeoff();

                    // CCC = 100 (4) -> Auto-landing
                    else if (link_system_byte == 0b100 && start > 0)
                        link_waypoint_step = 7;

                    // CCC = 110 (6) -> Land (turn off the motors)
                    else if (link_system_byte == 0b110)
                        link_check_and_turnoff_motors();

                    // CCC = 111 (7) -> Abort (FTS)
                    else if (link_system_byte == 0b111 && link_system_data == 0b1111)
                        liberty_x_fts();

                    // Clear direct control flag if CCC is not 001 (1)
                    if (link_system_byte != 0b001)
                        link_direct_control = 0;
                }

                // If link_system_byte < 128 -> P=1 -> command mode
                else {
                    // Call the anti-collision function if the direct control mode was previously used
                    if (link_direct_control)
                        direct_control_abort();

                    // Parse new waypoint latitude
                    waypoints_lat[link_system_data] = (int32_t)link_buffer[3] | (int32_t)link_buffer[2] << 8 | (int32_t)link_buffer[1] << 16 | (int32_t)link_buffer[0] << 24;
                    
                    // Parse new waypoint longitude
                    waypoints_lon[link_system_data] = (int32_t)link_buffer[7] | (int32_t)link_buffer[6] << 8 | (int32_t)link_buffer[5] << 16 | (int32_t)link_buffer[4] << 24;
                    
                    // Parse new waypoint command
                    waypoints_command[link_system_data] = link_system_byte;

                    // Request waypoint recalculation if not in auto-landing mode
                    if (link_waypoint_step > 3 && link_waypoint_step < 7)
                        link_waypoint_step = 3;

                    // Clear direct control flag
                    link_direct_control = 0;
                }
            }
        }
        else {
            // Store data bytes
            link_byte_previous = link_buffer[link_buffer_counter];
            link_buffer_counter++;

            // Reset buffer on overflow
            if (link_buffer_counter > 11)link_buffer_counter = 0;
        }
    }
}

#endif
