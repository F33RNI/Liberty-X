/*
 * Copyright (C) 2022 Fern Lane, Liberty-X Flight controller
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
 * The Liberty-X project started as a fork of the YMFC-32 project by Joop Brokking
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

#ifndef PID_H
#define PID_H

/******************************/
/*            Roll            */
/******************************/
// Roll P-controller (default = 1.3)
const float PID_ROLL_P PROGMEM = 3.8;

// Roll I-controller (default = 0.04)
const float PID_ROLL_I PROGMEM = 0.087;

// Roll D-controller (default = 18.0)
const float PID_ROLL_D PROGMEM = 47.0;

// Maximum output of the PID - controller (+ / -)
const float PID_ROLL_MAX PROGMEM = 400;


/*******************************/
/*            Pitch            */
/*******************************/
// Pitch P-controller (default = 1.3)
const float PID_PITCH_P PROGMEM = 3.8;

// Pitch I-controller (default = 0.04)
const float PID_PITCH_I PROGMEM = 0.087;

// Pitch D-controller (default = 18.0)
const float PID_PITCH_D PROGMEM = 47.0;

// Maximum output of the PID - controller (+ / -)
const float PID_PITCH_MAX PROGMEM = 400;


/*****************************/
/*            Yaw            */
/*****************************/
// Yaw P-controller (default = 4.0)
const float PID_YAW_P PROGMEM = 15.2;

// Yaw I-controller (default = 0.02)
const float PID_YAW_I PROGMEM = 0.2;

// Yaw D-controller (default = 0.0)
const float PID_YAW_D PROGMEM = 0.0;

// Maximum output of the PID - controller (+ / -)
const float PID_YAW_MAX PROGMEM = 400;


/**********************************/
/*            Altitude            */
/**********************************/
// Altitude P-controller (default = 1.4)
const float PID_ALT_P PROGMEM = 1.8;

// Altitude I-controller (default = 0.002)
const float PID_ALT_I PROGMEM = 0.0014;

// Altitude D-controller (default = 7.5)
const float PID_ALT_D PROGMEM = 14.0;

// Maximum output of the PID - controller (+ / -)
const float PID_ALT_MAX PROGMEM = 200;


/*****************************/
/*            GPS            */
/*****************************/
// GPS P-controller (default = 3.4)
const float PID_GPS_P PROGMEM = 3.4f;

// GPS rotating-memory D-controller (default = 7.6)
const float PID_GPS_D PROGMEM = 7.6f;

// Maximum output of the PID - controller (+ / -)
const float PID_GPS_MAX PROGMEM = 300;


#if (defined(SONARUS) && defined(LIBERTY_LINK))
/*********************************/
/*            Sonarus            */
/*********************************/
// Sonarus P-controller (default = 0.24)
const float PID_SONARUS_P PROGMEM = 0.22f;

// Sonarus I-controller (default = 0.0006)
const float PID_SONARUS_I PROGMEM = 0.0006f;

// Sonarus D-controller (default = 22.)
const float PID_SONARUS_D PROGMEM = 24.f;

// Maximum output of the PID - controller (+ / -)
const float PID_SONARUS_MAX PROGMEM = 150;
#endif


#ifdef SONARUS_COLLISION_PROTECTION
/******************************************************/
/*            Sonarus collision protection            */
/******************************************************/
// Sonarus collision protection P-controller (default = 0.6)
const float SONARUS_PROTECTION_P PROGMEM = 0.6f;

// Sonarus collision protection D-controller (default = 4.)
const float SONARUS_PROTECTION_D PROGMEM = 4.f;

// Maximum output of the PD - controller (+ / -)
const float SONARUS_PROTECTION_MAX PROGMEM = 200;
#endif


#ifdef LIBERTY_LINK
/*************************************************/
/*            Waypoint yaw correction            */
/*************************************************/
// Only P correction (default = 4)
const float WAYP_YAW_CORRECTION_TERM PROGMEM = 4.f;

// Max P correction (default = 150)
const float WAYP_YAW_CORRECTION_MAX PROGMEM = 100.f;

#endif

#endif
