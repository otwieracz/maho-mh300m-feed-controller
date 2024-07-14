# Principle of operation
1. Hardware
- MCP41HVX1 digital potentiometer is used to control the output voltage.
2. Setup
	1. Output calibration.
        - Set the output scale to full range (0-10V), control the output voltage with the potentiometer.
		- Set the potentiometer to minimum value where stable feed is achieved.
		- Store the 15bit value in EEPROM as `dac_min`.
		- Set the potentiometer to value where the output value, measured at the Parker 512C tach input, is `4.16V`.
		- Store the 15bit value in EEPROM as `dac_500`.
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

# Calibration Process

## Overview

This document provides a step-by-step guide to calibrating your device using the provided code. The calibration process involves setting up the digital potentiometer and ADC steps. During the process, the LED will blink specific messages to guide you through each step. Follow the LED messages carefully to ensure accurate calibration.

## LED Messages

- `..` (BLINKMSG_CONFIG_OK): Configuration is complete and successful.
- `-.` (BLINKMSG_SETUP_DIGIPOT_MIN): Calibrating the digital potentiometer to the minimum value.
- `.-` (BLINKMSG_SETUP_DIGIPOT_500): Calibrating the digital potentiometer to the 500mm/min feed rate.
- `-.-..` (BLINKMSG_SETUP_ADC_SETPOINT): Calibrating the ADC setpoint.

## Calibration Steps

1. **Calibrate Digital Potentiometer to Minimum Value**
    - The LED will blink `-..-.` indicating you should set the feed dial to the value that results in the slowest stable feed in all axes of the machine.
    - The device will automatically read the analog input for 30 seconds and adjust the digital potentiometer to the minimum value.

2. **Calibrate Digital Potentiometer to 500mm/min Feed Rate**
    - The LED will blink `-..--` indicating you should set the feed dial so the feed is precisely 500mm/min. Ensure the voltage setpoint reads 4.16V.
    - The device will automatically read the analog input for 30 seconds and adjust the digital potentiometer to the 500mm/min value.

3. **Calibrate ADC Steps**
    - For each step, the LED will blink `-.-..` indicating you should set the feed dial to the subsequent setpoint, starting from 0.
    - The device will read the analog input for 5.5 seconds for each step and store the ADC setpoint value.

4. **Complete Configuration**
    - Once all steps are calibrated, the LED will blink `.....` indicating the configuration is complete and successful.
    - The configuration data will be stored in the EEPROM for future use.

## Important Notes

- Make sure to follow the LED messages accurately and set the feed dial as instructed.
- The calibration process involves several steps, each requiring a specific feed dial setting. Take your time to ensure each step is done correctly.
- The calibration values will be stored in the EEPROM, so you do not need to recalibrate unless you make changes to your hardware setup.

By following this guide, you will ensure that your device is accurately calibrated for optimal performance. If you encounter any issues during the calibration process, refer to this document and ensure each step is followed precisely.