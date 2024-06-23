#include <Arduino.h>
#include <DFRobot_GP8XXX.h>
#include <EEPROM.h>

DFRobot_GP8211S dac;

#define ADC_MAX 1023
#define DAC_MAX 32767

#define ASCII_CR 13
#define ASCII_SPACE 32

void clear_line(int len) {
  for (int i = 0; i < len; i++) {
    Serial.write(ASCII_SPACE);
  }
  Serial.write(ASCII_CR);
}

#define ANALOG_PIN A0
#define N_STEPS 15 // Number of steps in the conversion table

#define CONFIG_VERSION 1
struct EepromConfig {
  int version;
  int dac_min; // Minimum DAC value for stable feed rate (15bit)
  int dac_500; // DAC value for 500mm/min feed rate - 4.16V measured at controller (15bit)
  int adc_setpoint[N_STEPS]; // ADC setpoint for each step in the conversion table
};

// ADC setpoints (feed rate) for each step in the conversion table
// note: 12mm/min is used instead of 12.5mm/min to avoid floating point arithmetic
int adc_steps[N_STEPS] = {0, 12, 20, 31, 40, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500};

// output uV per mm/min (500mm/min = 4.16V, 1mm/min = 0.00832V, 0.00832V * 1000 * 1000 = 8320uV)
int uv_per_mmmin = 8320;

// DAC (15bit) values for 1V: 32767 / 10 = 3276.7 (will be truncated to 3276)
int dac_1v = DAC_MAX / 10;

// Config - unititialized, to be read from EEPROM
EepromConfig config;

//
// Setup functions
//

// Read config from EEPROM
bool read_config(EepromConfig *config) {
  int addr = 0;
  EepromConfig eeprom_config;
  EEPROM.get(addr, eeprom_config);
  if (eeprom_config.version == CONFIG_VERSION) {
    *config = eeprom_config;
    return true;
  } else {
    Serial.println("Invalid configuration");
    return false;
  }
}

// Write empty config to EEPROM with version 0, enforcing reinitialization
void reset_eeprom_config() {
  EepromConfig default_config = {
    .version = 0,
    .dac_min = 0,
    .dac_500 = 0,
    .adc_setpoint = {0}
  };
  EEPROM.put(0, default_config);
}

// Setup DAC
void setup_dac() {
  Serial.println("setting up dac");
  dac.setDACOutRange(dac.eOutputRange10V);
}

//
// Calibration procedures
//

// Calibrate DAC
int calibrate_dac(char *name, int wait_time) {
  // Keep reading the analog input for CALIBRATION_WAIT ms
  unsigned long start = millis();
  while (millis() - start < wait_time) {
    unsigned long elapsed = millis() - start;
    unsigned long remaining = (wait_time - elapsed) / 1000;
    int adc_in = analogRead(ANALOG_PIN);
    int dac_out = map(adc_in, 0, ADC_MAX, 0, DAC_MAX);
    char buf[32];
    sprintf(buf, "config.%s: %5d (%2lus ...)", name, dac_out, remaining);
    Serial.print(buf);
    dac.setDACOutVoltage(dac_out);
    delay(100);
    Serial.write(ASCII_CR); // Carriage return
  }
  clear_line(32);
  char buf[32];
  int val = analogRead(ANALOG_PIN);
  int dac_out = map(val, 0, ADC_MAX, 0, DAC_MAX);
  sprintf(buf, "config.%s: %5d", name, dac_out); // extra spaces to clear previous output
  Serial.println(buf);
  return dac_out;
}

void initialize_config() {
  Serial.println("initializing configuration");
  int min = calibrate_dac("dac_min", 5500);
  int dac_500 = calibrate_dac("dac_500", 5500);
}

void setup() {
  Serial.begin(9600);
  setup_dac();
  if(!read_config(&config)) {
    initialize_config();
  }

}


void loop() {
  int val = analogRead(ANALOG_PIN);
  delay(200);
}