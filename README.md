## Principle of operation
1. Hardware
- DFR1071 (Gravity: GP8211 I2C 15-bit DAC Module (0-5V/10V)
2. Setup
	1. Output calibration.
        - Set the output scale to full range (0-10V), control the output voltage with the potentiometer.
		- Set the potentiometer to minimum value where stable feed is achieved.
		- Store the 15bit value in EEPROM as `min_output_value`.
		- Set the potentiometer to value where the output value, measured at the Parker 512C tach input, is `4.16V`.
		- Store the 15bit value in EEPROM as `max_output_value`.
	2. Input calibration
		- Repeatedly ask for setting the input endpoint for every of the pre-marked positions (0, 12.5, 31, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500)
		- For each position, store input ADC readout (10bit, 0-1023) in EEPROM
	3. As a result, we should have:
		- `minimum_output_value` maintaining stable feed
		- `maximum_output_value`, the output value for `500`setting
		- Expected `input` value for every `output` setting (0, 12.5, 31, ..., 500)
	4. Store above information in EEPROM.
3. Operation
	- Read the input value (10bit)
	- Find a the `input` range where the read value fits between (where `input` reading fits between two of the pre-defined setpoints) - two 10bit numbers
	- Find a matching `output` range by index - two numbers, from 0 to 500.
	- map input value range to output value range.
	- Convert `output` value (0-500) to `raw_output_value` (15bit) - `100 output = maximum_output_value/5`
	- If `raw_output` is less than `minimum_output_value`, return `0` as `raw_output_value`