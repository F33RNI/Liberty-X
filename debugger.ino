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
		DEBUG_SERIAL.print(DEBUG_VAR_1);
		DEBUG_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_2
		// Second variable
		DEBUG_SERIAL.print(DEBUG_VAR_2);
		DEBUG_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_3
		// Third variable
		DEBUG_SERIAL.print(DEBUG_VAR_3);
		DEBUG_SERIAL.print('\t');
#endif

#ifdef DEBUG_VAR_4
		// Fourth variable
		DEBUG_SERIAL.print(DEBUG_VAR_4);
		DEBUG_SERIAL.print('\t');
#endif

#if defined(DEBUG_VAR_1) || defined(DEBUG_VAR_2) || defined(DEBUG_VAR_3) || defined(DEBUG_VAR_4)
		// New line
		DEBUG_SERIAL.println();
#endif
	}
}

#endif

