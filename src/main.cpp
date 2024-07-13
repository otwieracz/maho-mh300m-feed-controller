#include <Arduino.h>
#include <EEPROM.h>
#include <LowPower.h>
#include <MCP41HVX1.h>
#include <config.h>
#include <SPI.h>
#include <blinkmsg.h>

MCP41HVX1 digipot(CS_PIN, SHDN_PIN, MCP41HVX1_PIN_NOT_CONFIGURED);

#define ADC_MAX 1023
#define DIGIPOT_MAX 255

#define N_STEPS 16    // Number of steps in the conversion table
#define RAPID 1200    // Rapid feed rate in mm/min
#define CREEP_DIV 10  // Creep feed rate divisor

#define CONFIG_VERSION 2
struct EepromConfig {
  int version;
  int digipot_min;  // Minimum digipot value for stable feed rate (15bit)
  int digipot_500;  // digipot value for 500mm/min feed rate - 4.16V measured at
                    // controller (15bit)
  int adc_setpoint[N_STEPS];  // ADC setpoint for each step in the conversion
                              // table
};

// ADC setpoints (feed rate) for each step in the conversion table
// note: 12mm/min is used instead of 12.5mm/min to avoid floating point
// arithmetic
int feed_table[N_STEPS] = {0,   12,  20,  31,  40,  50,  63,  80,
                           100, 125, 160, 200, 250, 315, 400, 500};

// Config - unititialized, to be read from EEPROM
EepromConfig config;

//
// Setup functions
//

// Reset configuration
void reset_eeprom_config(int addr) {
  EepromConfig default_config = {
      .version = 0, .digipot_min = 0, .digipot_500 = 0, .adc_setpoint = {0}};
  EEPROM.put(addr, default_config);
}

// Read config from EEPROM
bool read_config(EepromConfig *config) {
  int addr = 0;

  // Reset configuration if CLEAR_EEPROM button is pressed
  if (digitalRead(CLEAR_EEPROM_BTN_PIN) == LOW) {
    Serial.println("resetting configuration");
    reset_eeprom_config(addr);
    return false;
  }

  EepromConfig eeprom_config;
  EEPROM.get(addr, eeprom_config);
  if (eeprom_config.version == CONFIG_VERSION) {
    Serial.println("valid config");
    blinkmsg(BLINKMSG_CONFIG_OK);
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
      .version = 0, .digipot_min = 0, .digipot_500 = 0, .adc_setpoint = {0}};
  EEPROM.put(0, default_config);
}

// Setup buttons
void setup_buttons() {
  Serial.println("setting up buttons");
  pinMode(RAPID_BTN_PIN, INPUT_PULLUP);
  pinMode(CREEP_BTN_PIN, INPUT_PULLUP);
  pinMode(CLEAR_EEPROM_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

//
// Calibration procedures
//

// Calibrate digital potentionmeter
int calibrate_digipot(const char *name, unsigned int wait_time) {
  Serial.print("config.");
  Serial.print(name);
  Serial.print(": ");

  // Keep reading the analog input for wait_time ms
  unsigned long start = millis();
  int digipot_out = 0;
  while (millis() - start < wait_time) {
    int adc_in = analogRead(ANALOG_PIN);
    digipot_out = map(adc_in, 0, ADC_MAX, 0, DIGIPOT_MAX);
    digipot.WiperSetPosition(digipot_out);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  }

  Serial.println(digipot_out);

  return digipot_out;
}

// Calibrate ADC steps
//
void calibrate_adc_step(unsigned int step, int *buf, unsigned int wait_time) {
  int feed = feed_table[step];

  Serial.print("config.feed_");
  Serial.print(feed);
  Serial.print(": ");

  // Keep reading the analog input for wait_time ms
  unsigned long start = millis();
  int adc_in = 0;
  while (millis() - start < wait_time) {
    adc_in = analogRead(ANALOG_PIN);
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  }

  buf[step] = adc_in;

  Serial.println(adc_in);
}

void initialize_config() {
  Serial.println("initializing configuration");
  blinkmsg(BLINKMSG_SETUP);
  blinkmsg(BLINKMSG_PAUSE);
  blinkmsg(BLINKMSG_SETUP_DIGIPOT_MIN);
  int digipot_min = calibrate_digipot("digipot_min", 30000);
  blinkmsg(BLINKMSG_SETUP_DIGIPOT_500);
  int digipot_500 = calibrate_digipot("digipot_500", 30000);

  EepromConfig new_config = {.version = CONFIG_VERSION,
                             .digipot_min = digipot_min,
                             .digipot_500 = digipot_500,
                             .adc_setpoint = {0}};

  for (int i = 0; i < N_STEPS; i++) {
    blinkmsg(BLINKMSG_SETUP_ADC_SETPOINT);
    calibrate_adc_step(i, new_config.adc_setpoint, 5500);
  }

  blinkmsg(BLINKMSG_CONFIG_OK);
  EEPROM.put(0, new_config);
}

// Value mapping
//

// For given ADC input range, return the id of the lower bound
// of the range that contains the value
int find_range(int adc_value) {
  // Example:
  // adc_setpoint = {0, 100, 200, 300}
  // adc_value = 150
  // [1]:100 < adc_value:150 <= [2]:200
  // return 1

  // TODO: handle less than 0 and greater than N_STEPS-1
  for (int i = 0; i < N_STEPS - 1; i++) {
    if (config.adc_setpoint[i] < adc_value &&
        adc_value <= config.adc_setpoint[i + 1]) {
      return i;
    }
  }
  return 0;
}

// For given ADC value, return the feedrate in mm/min
// based on the conversion table and the current setpoint
// and the state of the rapid and creep buttons
int feedrate() {
  int adc_value = analogRead(ANALOG_PIN);
  int range_id = find_range(adc_value);

  bool rapid = digitalRead(RAPID_BTN_PIN) == LOW;
  bool creep = digitalRead(CREEP_BTN_PIN) == LOW;

  /* Round adc values lower than 12 (second step, first after 0) to 0*/
  if (adc_value < config.adc_setpoint[1]) {
    return 0;
  }

  int feedrate = 0;

  if (rapid) {
    feedrate = RAPID;
  } else if (adc_value == 0) {
    feedrate = 0;
  } else {
    feedrate = map(adc_value, config.adc_setpoint[range_id],
                   config.adc_setpoint[range_id + 1], feed_table[range_id],
                   feed_table[range_id + 1]);
  }

  if (creep && !rapid) {
    feedrate = feedrate / CREEP_DIV;
  }
  return feedrate;
}

// Convert feedrate in mm/min to digipot value
// based on the current configuration
// If reported feedrate is non-zero, but the calculated digipot value is below
// the minimum, return the minimum digipot value to ensure stable operation
long int feedrate_to_digipot(int feedrate) {
  long feed_digipot = (long)feedrate * (long)config.digipot_500 / 500;

  if (feedrate == 0) {
    return 0;
  } else if (feed_digipot > DIGIPOT_MAX) {
    return DIGIPOT_MAX;
  } else if (feed_digipot > 0 && feed_digipot < config.digipot_min) {
    return config.digipot_min;
  } else {
    return feed_digipot;
  }
}

bool setup_complete = false;

void setup() {
  Serial.begin(9600);
  setup_buttons();

  if (!read_config(&config)) {
    initialize_config();
    read_config(&config);
  }

  Serial.flush();
}

void loop() {
  int feedrate_val = feedrate();
  long int digipot_value = feedrate_to_digipot(feedrate_val);
  long int mv = map(digipot_value, 0, DIGIPOT_MAX, 0, 10000);

  if (digipot_value > 0) {
    digipot.ResistorNetworkEnable();
    digipot.WiperSetPosition(digipot_value);
  } else {
    digipot.ResistorNetworkDisable();
  }

  int v = mv / 1000;
  int v_mod = mv % 1000;
  char buf[64];
  sprintf(buf, "fr: %4dmm/min / %1d.%02dV", feedrate_val, v, v_mod / 10);
  Serial.println(buf);
  Serial.flush();

  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
}