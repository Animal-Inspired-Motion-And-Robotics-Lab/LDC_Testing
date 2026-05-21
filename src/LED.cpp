#include "LED.h"

#include <Arduino.h>

namespace {

static int gLedPin = LED_BUILTIN;
static bool gActiveHigh = true;

inline uint8_t ledOnLevel(void) {
  return gActiveHigh ? HIGH : LOW;
}

inline uint8_t ledOffLevel(void) {
  return gActiveHigh ? LOW : HIGH;
}

}  // namespace

void ledInit(int pin, bool active_high) {
  gLedPin = pin;
  gActiveHigh = active_high;

  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOffLevel());
}

void ledOn(void) {
  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOnLevel());
}

void ledOff(void) {
  pinMode(gLedPin, OUTPUT);
  digitalWrite(gLedPin, ledOffLevel());
}

void ledFlash(uint8_t count, uint32_t ms_per_flash) {
  for (uint8_t i = 0; i < count; ++i) {
    ledOn();
    delay(ms_per_flash);
    ledOff();
    delay(ms_per_flash);
  }
}
