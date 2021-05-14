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

/// <summary>
/// Calculates vertical acceleration over a longer period via a rotating memory.
/// </summary>
void vertical_acceleration(void) {
    acc_z_average_short_rotating_mem_location++;
    if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

    acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
    acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
    acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

    if (acc_z_average_short_rotating_mem_location == 0) {
        acc_z_average_long_rotating_mem_location++;

        if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

        acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
        acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
        acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
    }
    acc_z_average_total = acc_z_average_long_total / 50;


    acc_alt_integrated += acc_total_vector - acc_z_average_total;
    if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
        if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
            if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
            else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
    }
}
