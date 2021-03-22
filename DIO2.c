#ifdef __AVR__
#include "platform.h"
#include "config_pins.h"
#ifdef USEDIO

#include "Arduino.h"

#ifndef _LIB_SAM_

// Standard headers for AVR
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define ARDUINO2_MAIN

#include "DIO2.h"


// Internal worker for pinMode2. It is called if the pin or mode
// are not known at compile time.
// The code is the same as in the fast version, just disables interrupts
// before modifying the registers.
void internal_pinMode2(GPIO_pin_t pin, uint8_t mode)
{
  if ( mode == OUTPUT )
  {
    GPIO2_ATOMIC_BEGIN
    GPIO_DDR_REG(pin) |= GPIO_PIN_MASK(pin);
    GPIO2_ATOMIC_END
  }
  else
  {
    if ( mode == INPUT_PULLUP )
    {
      GPIO2_ATOMIC_BEGIN
      GPIO_DDR_REG(pin) &= ~GPIO_PIN_MASK(pin);
      GPIO_PORT_REG(pin) |= GPIO_PIN_MASK(pin);
      GPIO2_ATOMIC_END
    }
    else
    {
      // input mode without pull-up
      GPIO2_ATOMIC_BEGIN
      GPIO_DDR_REG(pin) &= ~GPIO_PIN_MASK(pin);
      GPIO_PORT_REG(pin) &= ~GPIO_PIN_MASK(pin);
      GPIO2_ATOMIC_END
    }
  }
}

// Internal worker for digitalRead2f.
// It is called if the pin in not known at compile time.
uint8_t internal_digitalRead2(GPIO_pin_t pin)
{
  if ((GPIO_PIN_REG(pin) & GPIO_PIN_MASK(pin)) != 0)
    return HIGH;
  else
    return LOW;
}

// Internal worker for digitalWrite2f.
// It is called if the pin or value are not known at compile time.
// The code is the same as in the fast version, just disables interrupts
// before modifying the registers.
void internal_digitalWrite2(GPIO_pin_t pin, uint8_t value)
{
  if ( value == 0 )
  {
    GPIO2_ATOMIC_BEGIN
    GPIO_PORT_REG(pin) &= ~GPIO_PIN_MASK(pin);
    GPIO2_ATOMIC_END
  }
  else
  {
    GPIO2_ATOMIC_BEGIN
    GPIO_PORT_REG(pin) |= GPIO_PIN_MASK(pin);
    GPIO2_ATOMIC_END
  }
}

int GPIO_to_Arduino_pin(GPIO_pin_t inPin)
{
#ifdef GPIO2_PREFER_SPEED
  switch ((uint32_t)inPin)
  {
      GPIO_TO_ARDUINO;
  }
#else
  int i;
  for (i = 0; i < GPIO_PINS_NUMBER; i++)
  {
    if (inPin == GPIO_GET_PINDEF(i))
      return i;
  }
#endif
  return -1;
}
#endif

#endif // usedio

#endif

