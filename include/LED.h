#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include <stdint.h>

// Initialize LED control with a pin number and polarity.
// active_high=true means HIGH turns LED on.
void ledInit(int pin, bool active_high);

// Turn LED on/off using the configured pin and polarity.
void ledOn(void);
void ledOff(void);

// Blink LED count times with ms_per_flash milliseconds on and off.
void ledFlash(uint8_t count, uint32_t ms_per_flash);

#endif
