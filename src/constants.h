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

#ifndef CONSTANTS_H
#define CONSTANTS_H

// The loop frequency is 250 Hz. Changing it may cause flyaway or even injury
const uint32_t LOOP_PERIOD PROGMEM = 4000;
const uint32_t MAX_ALLOWED_LOOP_PERIOD PROGMEM = 4050;

// Hardware constants
const uint8_t IMU_ADDRESS PROGMEM = 0x68;
const uint8_t BAROMETER_ADDRESS PROGMEM = 0x77;
const uint8_t COMPASS_ADDRESS PROGMEM = 0x1E;
const uint8_t VOLTMETER_PIN PROGMEM = 4;

// GPS UBLOX predefined messages for setup
const uint8_t GPS_DISABLE_GPGSV[11] PROGMEM = { 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15 };
const uint8_t GPS_SET_TO_5HZ[14] PROGMEM = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A };
const uint8_t GPS_SET_TO_57KBPS[28] PROGMEM = { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 
0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1 };

#endif
