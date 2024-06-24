#include <Arduino.h>
#include <DFRobot_GP8XXX.h>
#include <EEPROM.h>

DFRobot_GP8211S dac;

#define ADC_MAX 1023
#define DAC_MAX 32767

#define ASCII_BSPACE 8
#define ASCII_CR 13
#define ASCII_SPACE 32

void clear_line(int len) {
  for (int i = 0; i < len; i++) {
    Serial.write(ASCII_SPACE);
  }
  Serial.write(ASCII_CR);
}

void backspace(int len) {
  for (int i = 0; i < len; i++) {
    Serial.write(ASCII_BSPACE);
  }
}

#define ANALOG_PIN A0
#define N_STEPS 16 // Number of steps in the conversion table

#define CONFIG_VERSION 1
struct EepromConfig {
  int version;
  int dac_min; // Minimum DAC value for stable feed rate (15bit)
  int dac_500; // DAC value for 500mm/min feed rate - 4.16V measured at controller (15bit)
  int adc_setpoint[N_STEPS]; // ADC setpoint for each step in the conversion table
};

// ADC setpoints (feed rate) for each step in the conversion table
// note: 12mm/min is used instead of 12.5mm/min to avoid floating point arithmetic
int adc_steps[N_STEPS] = {0, 12, 20, 31, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, 500};

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
    Serial.println("valid config");
    *config = eeprom_config;
    return true;
  } else {
    Serial.println("invalid configuration");
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
int calibrate_dac(const char *name, unsigned int wait_time) {
  Serial.print("config.");
  Serial.print(name);
  Serial.print(": ");

  // Keep reading the analog input for wait_time ms
  unsigned long start = millis();
  int dac_out = 0;
  while (millis() - start < wait_time) {
    int adc_in = analogRead(ANALOG_PIN);
    dac_out = map(adc_in, 0, ADC_MAX, 0, DAC_MAX);
    dac.setDACOutVoltage(dac_out);
    delay(100);
  }

  Serial.println(dac_out);

  return dac_out;
}

// Calibrate ADC steps
// 
void calibrate_adc_step(unsigned int step, int *buf, unsigned int wait_time) {
  int feed = adc_steps[step];

  Serial.print("config.feed_");
  Serial.print(feed);
  Serial.print(": ");

  // Keep reading the analog input for wait_time ms
  unsigned long start = millis();
  int adc_in = 0;
  while (millis() - start < wait_time) {
    adc_in = analogRead(ANALOG_PIN);
    delay(100);
  }

  buf[step] = adc_in;

  Serial.println(adc_in);
}

void initialize_config() {
  Serial.println("initializing configuration");
  int dac_min = calibrate_dac("dac_min", 5500);
  int dac_500 = calibrate_dac("dac_500", 5500);

  EepromConfig new_config = {
    .version = CONFIG_VERSION,
    .dac_min = dac_min,
    .dac_500 = dac_500,
    .adc_setpoint = {0}
  };

  for(int i = 0; i < N_STEPS; i++) {
    calibrate_adc_step(i, config.adc_setpoint, 5500);
  }

  EEPROM.put(0, new_config);
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