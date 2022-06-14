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

/// <summary>
/// Starts the voltmeter. Because filter initialization is required
/// </summary>
void voltmeter_setup(void) {
	// Calculate voltage without filter
	battery_voltage = ((float)analogRead(4) / VOLTAGE_ADC_DIVIDER);
}

/// <summary>
/// Regular coltage calculation (with filter)
/// </summary>
void voltmeter(void) {
	// Calculate voltage
	battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / VOLTAGE_ADC_DIVIDER) * 0.08;

	// Check voltage
	if (battery_voltage > 6.0 && battery_voltage < BATTERY_WARNING) {
		// Set error to ERROR_LOW_BATTERY if currently no error
		if (!error)
			error = ERROR_LOW_BATTERY;

		// Start auto-landing sequence on low battery
#ifdef AUTO_LANDING_LOW_VOLTAGE
		if (!auto_landing_step && start > 0)
			auto_landing_step = 1;
#endif
	}
}
