#include <Arduino.h>
#include <blinkmsg.h>
#include <config.h>

#define MESSAGE_LENGTH 500
#define MESSAGE_SHORT 25 
#define MESSAGE_LONG 400

//
// LED messages
//

void blink_led(const char *msg) {
  for (int i = 0; i < 5; i++) {
    if (msg[i] == '.') {
      digitalWrite(LED_PIN, LOW);
      delay(MESSAGE_SHORT);
      digitalWrite(LED_PIN, HIGH);
      delay(MESSAGE_LENGTH - MESSAGE_SHORT);
    } else if (msg[i] == '-') {
      digitalWrite(LED_PIN, LOW);
      delay(MESSAGE_LONG);
      digitalWrite(LED_PIN, HIGH);
      delay(MESSAGE_LENGTH - MESSAGE_LONG);
    }
  }
}

void blinkmsg(int blinkmsg) {
  switch (blinkmsg) {
    case BLINKMSG_PAUSE:
      delay(MESSAGE_LENGTH);
      break;
    case BLINKMSG_CONFIG_OK:
      blink_led(".....");
      break;
    case BLINKMSG_SETUP:
      blink_led("-...-");
      break;
    case BLINKMSG_SETUP_DIGIPOT_MIN:
      blink_led("-..-.");
      break;
    case BLINKMSG_SETUP_DIGIPOT_500:
      blink_led("-..--");
      break;
    case BLINKMSG_SETUP_ADC_SETPOINT:
      blink_led("-.-..");
      break;
  }
}
