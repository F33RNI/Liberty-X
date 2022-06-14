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

#ifdef LUX_METER

/// <summary>
/// Initializes BH1750 Lux meter
/// </summary>
void lux_meter_setup(void) {
	// Check if the BH1750 is responding
	HWire.beginTransmission(LUX_METER_ADDRESS);

	// Power on BH1750
	HWire.write(0x01);

	error = HWire.endTransmission();
	while (error) {
		// Stay in the loop because the lux meter did not responde
		error = ERROR_BOOT_LUX_METER;
		// Show curent error
		leds_error_signal();
		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Reset BH1750
	HWire.beginTransmission(LUX_METER_ADDRESS);
	HWire.write(0x07);
	HWire.endTransmission();

	// Power down BH1750
	HWire.beginTransmission(LUX_METER_ADDRESS);
	HWire.write(0x00);
	HWire.endTransmission();

	// Wait some time
	delay(100);

	// Power on BH1750
	HWire.beginTransmission(LUX_METER_ADDRESS);
	HWire.write(0x01);
	HWire.endTransmission();

	// Set MTreg to 31 (lowest sensitivity)
	HWire.beginTransmission(LUX_METER_ADDRESS);
	HWire.write(0x40);
	HWire.write(0x7F);
	HWire.endTransmission();

	// Select continuously L-resolution mode
	HWire.beginTransmission(LUX_METER_ADDRESS);
	HWire.write(0x13);
	HWire.endTransmission();

	// Wait some time to complete first measurement
	delay(100);
}


/// <summary>
/// Reads illumination from the BH1750 sensor
/// </summary>
void lux_meter(void) {
	// Increment counter every cycle
	lux_cycle_counter++;

	// Last cycle. Request LUX data
	if (lux_cycle_counter >= LUX_REQUST_CYCLES) {
		// Restart counter
		lux_cycle_counter = 0;

		// Request 2 bytes from sensor
		HWire.requestFrom(LUX_METER_ADDRESS, 2);

		// Read 2 bytes from sensor
		lux_raw_data = HWire.read() << 8 | HWire.read();

		// Convert to lux (divide by 0.54)
		lux_data = lux_raw_data / 0.54;

		// Compress value
		lux_data = pow(lux_data, 0.475);
		if (lux_data > 254.0)
			lux_data = 254.0;

		// Convert to sinle byte
		lux_sqrt_data = lux_data;
	}
}
#endif
