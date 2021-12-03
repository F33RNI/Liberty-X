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

#ifndef CONFIG_H
#define CONFIG_H

/*********************************/
/*            Compass            */
/*********************************/
// Declination between the magnetic and geographic north. (-4.89 in Panama, 11.0 in Moscow)
const float COMPASS_DECLINATION PROGMEM = -4.89;

// Pring compass calibration values to the serial port
//#define PRINT_COMPASS_CALIBRATION


/*********************************/
/*            Battery            */
/*********************************/
// At this voltage, error will be set to 1
const float BATTERY_WARNING PROGMEM = 10.4;

// Divider to convert raw ADC value to volts
const float VOLTAGE_ADC_DIVIDER PROGMEM = 109.88;


/********************************/
/*            Motors            */
/********************************/
// Disable motors spinning in flight. Useful for debug
//#define DISABLE_MOTORS

// Takeoff throttle set 0 to enable auto-takeoff
const int32_t MANUAL_TAKEOFF_THROTTLE PROGMEM = 1500; //1500

// Throttle in flight_mode = 1 will be passed through the exp() function to reduce the sharpness
// Excel formula (input: column A, output: column B):
// =IF(A1 < 1500, 1500 - EXP(ABS(A1-1500)/120.0) * 6.0, 1500 + EXP(ABS(A1-1500) / 120.0) * 6.0)
const float MANUAL_TROTTLE_EXP PROGMEM = 120.0;
const float MANUAL_TROTTLE_RATE PROGMEM = 6.0;

// Voltage drop compenstation factor
const float BATTERY_COMPENSATION PROGMEM = 65.0;

// IDLE speed (minimum speed) of the motors 
const uint16_t MOTOR_IDLE_SPEED PROGMEM = 1200;

// Takeoff detected when (acc_z_average_short_total / 25 - acc_total_vector_at_start) > AUTO_TAKEOFF_ACC_THRESHOLD
const int32_t AUTO_TAKEOFF_ACC_THRESHOLD PROGMEM = 800;


/*****************************/
/*            IMU            */
/*****************************/
// Level calibration value. Increasing causes moving to the right (>). Uncomment to overwrite the read from EEPROM
#define ACC_CALIBRATION_ROLL	-50

// Level calibration value. Increasing causes moving backward (\/). Uncomment to overwrite the read from EEPROM
#define ACC_CALIBRATION_PITCH	1160

// Pring level calibration values to the serial port
#define PRINT_LEVEL_CALIBRATION

// Pring gyro calibration values to the serial port
//#define PRINT_GYRO_CALIBRATION

// Number of gyro and acc calibartion reads
const uint16_t IMU_CALIBARTION_N PROGMEM = 2000;


/***********************************/
/*            Barometer            */
/***********************************/
// Set the altitude setpoint at groundlevel + approximately 1.5 meter
const float PRESSURE_TAKEOFF PROGMEM = 10;

// Manual pressure setpoint change factor (Increasing causes faster setpoint change)
const float PRESSURE_SP_FACTOR PROGMEM = 0.0003;

// Stabilize pressure in 1000 * 4ms = 4000ms
const uint16_t PRESSURE_STAB_N PROGMEM = 1000;


/*****************************/
/*            GPS            */
/*****************************/
// Baud rate of GPS mixer serial port
const uint32_t GPS_BAUD_RATE = 115200;

// If no data in 100 * 4ms = 400ms the gps will be considered lost
const uint8_t GPS_LOST_CYCLES PROGMEM = 100;

// Unique pair of suffix
const uint8_t GPS_SUFFIX_1 PROGMEM = 0xEE;
const uint8_t GPS_SUFFIX_2 PROGMEM = 0xEF;

// GPS every 5 program loops 5 x 4ms = 20ms
// 5 - means that between every two new coordinates 4 will be simulated (overall 20ms period)
const int16_t GPS_PREDICT_AFTER_CYCLES PROGMEM = 5;


/********************************************/
/*            In-flight adjuster            */
/********************************************/
// You can adjust this variable with channel_8
#define ADJUST_RULE				gimbal_pitch = channel_8


/********************************************************/
/*            Sonarus ultrasonic rangefinder            */
/********************************************************/
// Used to prevent collisions and to improve the automatic take-off and motors shutdown function (in Liberty-Link)
#define SONARUS

#ifdef SONARUS
// Whether to detect liftoff using sonar 2. Comment to determine takeoff based on acceleration only
//#define SONARUS_TAKEOFF_DETECTION

// Determine that the drone has taken off if the distance from the sonar has changed more than this number
#ifdef SONARUS_TAKEOFF_DETECTION
const uint16_t SONARUS_TAKEOFF_INCREMENT PROGMEM = 9;
#endif

// Request distance value every 20 * 4ms = 80ms
const uint8_t SONARUS_REQUST_CYCLES PROGMEM = 20;

// Speed of sound in m/s * 10000
const uint16_t SOUND_SPEED PROGMEM = 343;

// Minimum distance (in mm) at which collision protection begins (default: 1000)
const uint16_t SONARUS_SPRING_START PROGMEM = 1000;

// At what distance (in mm) the maximum pitch back will be set (default: 100)
const uint16_t SONARUS_SPRING_STOP PROGMEM = 100;

// Maximum backward pitch value (1500 + SONARUS_MAX_PITCH)
const uint16_t SONARUS_MAX_PITCH PROGMEM = 200;
#endif


/*************************************/
/*            Light meter            */
/*************************************/
// Used to dynamically change the camera exposure depending on the illumination of the ARUco marker
//#define LUX_METER

#ifdef LUX_METER
// Request LUX value every 25 * 4ms = 100ms
const uint8_t LUX_REQUST_CYCLES PROGMEM = 25;
#endif


/*****************************************************/
/*            Liberty-Link implementation            */
/*****************************************************/
#define LIBERTY_LINK

#ifdef LIBERTY_LINK

// ----- Communication -----
// Unique pair of ASCII symbols
const uint8_t LINK_SUFFIX_1 PROGMEM = 0xEE;
const uint8_t LINK_SUFFIX_2 PROGMEM = 0xEF;

// If no data in 125 * 4ms = 500ms the connection will be considered lost
const uint8_t LINK_LOST_CYCLES PROGMEM = 125;

// ----- Waypoints flight section -----
// Max GPS setpoint distance (at what maximum distance can the GPS setpoint mode and descent be immediately activated)
const int32_t GPS_SETPOINT_MAX_DISTANCE PROGMEM = 5;

// How many pascals to reduce the pressure (raise the altitude) before GPS flight
const float LINK_PRESSURE_ASCEND PROGMEM = 20;

// GPS waypoint move speed factor on short distances (default = 0.05, larger = faster)
const float WAYPOINT_GPS_MIN_FACTOR PROGMEM = 0.015;

// GPS waypoint move speed factor on large distances (default = 0.20, larger = faster)
const float WAYPOINT_GPS_MAX_FACTOR PROGMEM = 0.05;

// Altitude waypoint move speed term (default = 0.022, larger = faster)
const float WAYPOINT_ALTITUDE_TERM PROGMEM = 0.025;

// How many pascals to reduce the pressure (raise the altitude) when the direct control aborted
const float ABORT_PRESSURE_ASCEND PROGMEM = 20;

// ----- Pre-flight checks section -----
// The minimum number of satellites required for takeoff
const uint8_t LINK_MIN_NUM_SATS PROGMEM = 5;

// The minimum battery voltage required for takeoff
const float LINK_MIN_BAT_VOLTAGE PROGMEM = 11.0;

// ----- Sonarus section -----
#ifdef SONARUS
// Allow motor shutdown only when sonar_2_raw value is greater than 0 and lower than SONARUS_LINK_MTOF
// Comment to allow motor shutdown (with Liberty-Link command) at any height
#define SONARUS_MTOF_PROTECTION

// At what distance (in mm) the motors can be turned off (default: 500)
#ifdef SONARUS_MTOF_PROTECTION
const uint16_t SONARUS_LINK_MTOF PROGMEM = 500;
#endif

#endif
#endif


/***********************************/
/*            Telemetry            */
/***********************************/
#define TELEMETRY

#ifdef TELEMETRY
// Unique pair of ASCII symbols
const uint8_t TELEMETRY_SUFFIX_1 PROGMEM = 0xEE;
const uint8_t TELEMETRY_SUFFIX_2 PROGMEM = 0xEF;

// How many bytes will be transmitted at the same time (in liberty-link mode)
const uint8_t BURST_BYTES PROGMEM = 4;
#endif


/**********************************/
/*            Debugger            */
/**********************************/
//#define DEBUGGER

#ifdef DEBUGGER
// Send debug every 25 * 4ms = 100ms
const uint16_t DEBUG_SEND_CYCLES PROGMEM = 25;

// Variables to debug
//#define DEBUG_VAR_1				sonar_1_raw
//#define DEBUG_VAR_2				sonar_2_raw
//#define DEBUG_VAR_3				sonar_2_prev
//#define DEBUG_VAR_4				sonar_2_at_start
#endif


/**************************************/
/*            Serial ports            */
/**************************************/
// Telemetry, Liberty-Link and debugger port
#define TELEMETRY_SERIAL		Serial1

// GPS port
#define GPS_SERIAL				Serial2

// Telemetry, Liberty-Link and Debugger port baud rate
const uint32_t TELEMETRY_BAUDRATE PROGMEM = 115200;


/*************************************/
/*            WS2812 LEDs            */
/*************************************/
// LED colors
#define COLOR_CALIBRATION		63, 0, 255
#define COLOR_ERROR				255, 110, 0
#define COLOR_FRONT				0, 255, 0
#define COLOR_REAR				255, 0, 0
#define COLOR_FRONT_LINK		0, 127, 127
#define COLOR_REAR_LINK			127, 127, 0
#define COLOR_BLINK				255, 255, 255

// Change state every 63 * 4ms ~ 250ms for error signal
const uint16_t LEDS_ERROR_CYCLES PROGMEM = 63;

// Change state every 50 * 4ms = 200ms for flight mode signal
const uint16_t LEDS_FLIGHT_MODE_CYCLES PROGMEM = 50;

// Change state every 10 * 4ms = 40ms for idle rainbow sweep
const uint16_t LEDS_IDLE_CYCLES PROGMEM = 10;

// Blink in flight after every 250 * 4ms = 1000ms
const uint16_t LEDS_BLINK_CYCLES PROGMEM = 250;

#endif
