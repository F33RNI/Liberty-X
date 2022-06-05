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

#ifndef CONSTANTS_H
#define CONSTANTS_H

// The loop frequency is 250 Hz. Changing it may cause flyaway or even injury
const uint32_t LOOP_PERIOD PROGMEM = 4000;
const uint32_t MAX_ALLOWED_LOOP_PERIOD PROGMEM = 4050;

// Hardware constants
const uint8_t IMU_ADDRESS PROGMEM = 0x68;
const uint8_t BAROMETER_ADDRESS PROGMEM = 0x77;
const uint8_t COMPASS_ADDRESS PROGMEM = 0x1E;
#ifdef LUX_METER
const uint8_t LUX_METER_ADDRESS PROGMEM = 0x23;
#endif
#ifdef SONARUS
const uint8_t SONARUS_ADDRESS PROGMEM = 0xEE;
#endif
const uint8_t VOLTMETER_PIN PROGMEM = 4;

// Startup error codes
#define ERROR_BOOT_IMU					1
#define ERROR_BOOT_COMPASS				2
#define ERROR_BOOT_BAROMETER			3
#define ERROR_BOOT_LUX_METER			4
#define ERROR_BOOT_SONARUS				5
#define ERROR_BOOT_RC					6

// Error codes
#define ERROR_LOW_BATTERY				1
#define ERROR_LOOP_TIME					2
#define ERROR_GPS_LOST					3
#define ERROR_MANUAL_TAKEOFF			4
#define ERROR_TAKEOFF_NOT_DETECTED		5
#define ERROR_TAKEOFF_NOT_CALCULATED	6
#define ERROR_FTS						7
#define ERROR_SONARUS_TAKEOFF			8
#define ERROR_SONARUS_COLLISION			9


// Liberty-Way steps
#ifdef LIBERTY_LINK
#define LINK_STEP_IDLE			0
#define LINK_STEP_TAKEOFF		1
#define LINK_STEP_ASCENT		2
#define LINK_STEP_WAYP_CALC		3
#define LINK_STEP_GPS_WAYP		4
#define LINK_STEP_GPS_SETP		5
#define LINK_STEP_DESCENT		6
#define LINK_STEP_SONARUS		7
#define LINK_STEP_AFTER_SONARUS	8

// Liberty-Link waypoint commands (bits)
#define WAYP_CMD_BITS_SKIP				0b000
#define WAYP_CMD_BITS_DDC_NO_GPS_NO_DSC	0b001
#define WAYP_CMD_BITS_DDC_NO_DSC		0b010
#define WAYP_CMD_BITS_DDC				0b011
#define WAYP_CMD_BITS_FLY				0b100
#define WAYP_CMD_BITS_DESCEND			0b101
#define WAYP_CMD_BITS_PARCEL			0b110
#define WAYP_CMD_BITS_LAND				0b111

// Liberty-Link commands (bits)
#define CMD_BITS_IDLE				0b000
#define CMD_BITS_DDC				0b001
#define CMD_BITS_AUTO_TAKEOFF		0b010
#define CMD_BITS_AUTO_LAND			0b100
#define CMD_BITS_DDC_LAND			0b110
#define CMD_BITS_FTS				0b111

#endif

#endif
