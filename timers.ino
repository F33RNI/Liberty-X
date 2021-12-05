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

/// <summary>
/// Initializes system timers for PPM decoder, ESC output and gimbal control
/// </summary>
void timers_setup(void) {
	// PPM decoder
	Timer2.attachCompare1Interrupt(ppm_decoder);
	TIMER2_BASE->CR1 = TIMER_CR1_CEN;
	TIMER2_BASE->CR2 = 0;
	TIMER2_BASE->SMCR = 0;
	TIMER2_BASE->DIER = TIMER_DIER_CC1IE;
	TIMER2_BASE->EGR = 0;
	TIMER2_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
	TIMER2_BASE->CCMR2 = 0;
	TIMER2_BASE->CCER = TIMER_CCER_CC1E;

	//Detect falling edge
	//TIMER2_BASE->CCER |= TIMER_CCER_CC1P;

	//Detect rising edge
	TIMER2_BASE->CCER &= ~TIMER_CCER_CC1P;

	TIMER2_BASE->PSC = 71;
	TIMER2_BASE->ARR = 0xFFFF;
	TIMER2_BASE->DCR = 0;

	// Motors
	TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
	TIMER4_BASE->CR2 = 0;
	TIMER4_BASE->SMCR = 0;
	TIMER4_BASE->DIER = 0;
	TIMER4_BASE->EGR = 0;
	TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
	TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
	TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
	TIMER4_BASE->PSC = 71;
	TIMER4_BASE->ARR = 5000;
	TIMER4_BASE->DCR = 0;
	TIMER4_BASE->CCR1 = 1000;

	TIMER4_BASE->CCR1 = 1000;
	TIMER4_BASE->CCR2 = 1000;
	TIMER4_BASE->CCR3 = 1000;
	TIMER4_BASE->CCR4 = 1000;
	pinMode(PB6, PWM);
	pinMode(PB7, PWM);
	pinMode(PB8, PWM);
	pinMode(PB9, PWM);

	// Gimbal or latch mechanism
	TIMER3_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
	TIMER3_BASE->CR2 = 0;
	TIMER3_BASE->SMCR = 0;
	TIMER3_BASE->DIER = 0;
	TIMER3_BASE->EGR = 0;
	TIMER3_BASE->CCMR2 = (0b110 << 12) | TIMER_CCMR2_OC4PE;
	TIMER3_BASE->CCER = TIMER_CCER_CC4E;
	TIMER3_BASE->PSC = 71;
	TIMER3_BASE->ARR = 5000;
	TIMER3_BASE->DCR = 0;
	TIMER3_BASE->CCR4 = 2000;
	pinMode(PB1, PWM);
}
