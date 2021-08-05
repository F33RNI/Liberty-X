/// <summary>
/// Initilizes
/// </summary>
/// <param name=""></param>
void sonarus_setup(void) {
	// Check if the sonar system is responding
	HWire.beginTransmission(SONARUS_ADDRESS);

	// Power on sonars
	HWire.write(0x01);

	error = HWire.endTransmission();
	while (error != 0) {
		// Stay in the loop because the lux meter did not responde
		error = 5;
		// Show curent error
		leds_error_signal();
		// Simulate main loop
		delayMicroseconds(LOOP_PERIOD);
	}

	// Select continuously mode
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x02);
	HWire.write(0x01);
	HWire.endTransmission();

	// Select low-resolution (1-byte) mode (distances will be divided by 2)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x03);
	HWire.write(0x00);
	HWire.endTransmission();

	// Reset filter
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x04);
	HWire.write(0x00);
	HWire.endTransmission();

	// Set speed of sound
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x05);
	HWire.write(SOUND_SPEED >> 8);
	HWire.write(SOUND_SPEED & 0xFF);
	HWire.endTransmission();

	// Set distance correction for first sonar (-34.3) (-343 -> 0xFEA4)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x06);
	HWire.write(0xFE);
	HWire.write(0xA9);
	HWire.write(0x00);
	HWire.write(0x00);
	HWire.endTransmission();

	// Request first measurement
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x07);
	HWire.endTransmission();

	// Wait for first measurement to complete
	delay(100);

	// Set filter to 0.5 (50 -> 0x32)
	HWire.beginTransmission(SONARUS_ADDRESS);
	HWire.write(0x04);
	HWire.write(0x32);
	HWire.endTransmission();
}

void sonarus(void) {
	// Increment counter every cycle
	sonarus_cycle_counter++;

	// Last cycle. Request LUX data
	if (sonarus_cycle_counter >= SONARUS_REQUST_CYCLES) {
		// Restart counter
		sonarus_cycle_counter = 0;

		// Request 2 bytes from sonar (1-byte per sonars)
		HWire.requestFrom(SONARUS_ADDRESS, 2);

		// Read distances from both sonars (divided by 2)
		sonar_1_raw = HWire.read();
		sonar_2_raw = HWire.read();
	}
}
