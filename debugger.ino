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

#ifdef DEBUGGER

/// <summary>
/// Prints up to four variables to the serial port
/// </summary>
void debugger(void) {
	// Increment counter
	debugger_loop_counter++;

	if (debugger_loop_counter >= DEBUG_SEND_CYCLES) {
		// Reset counter
		debugger_loop_counter = 0;

#ifdef DEBUG_VAR_1
		// First variable
		TELEMETRY_SERIAL.print(DEBUG_VAR_1);
		TELEMETRY_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_2
		// Second variable
		TELEMETRY_SERIAL.print(DEBUG_VAR_2);
		TELEMETRY_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_3
		// Third variable
		TELEMETRY_SERIAL.print(DEBUG_VAR_3);
		TELEMETRY_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_4
		// Fourth variable
		TELEMETRY_SERIAL.print(DEBUG_VAR_4);
		TELEMETRY_SERIAL.print('\t');
#endif

		TELEMETRY_SERIAL.println();
	}
}

#endif

