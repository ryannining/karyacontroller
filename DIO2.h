/*
  DIO2.h

   Created on: 11. 2. 2014
       Author: Jan Dolinay
   Adapted by: Thierry Paris

    Alternate version of digital input/output for Arduino.
    This is common file included to user application.
    It includes the pins2_arduino.h with definitions specific to given
    Arduino variant (e.g. standard or mega).

  This is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef ARDUINO2_H_
#define ARDUINO2_H_

#ifdef ARDUINO_ARCH_SAM

// DUE version
// The due is probably fast enough to access ports very efficiency (and i dont know how to make better !) .
// So the functions are dummies, and conduct to original Arduino functions...

typedef int GPIO_pin_t;
#define pinMode2(P, M)      pinMode(P, M)
#define digitalWrite2(P, V)   digitalWrite(P, V)
#define digitalRead2(P)     digitalRead(P)

#define pinMode2f(P, M)     pinMode(P, M)
#define digitalWrite2f(P, V)  digitalWrite(P, V)
#define digitalRead2f(P)    digitalRead(P)

#define Arduino_to_GPIO_pin(P)  (P)
#define GPIO_to_Arduino_pin(P)  (P)

#define DP_INVALID  0xFFFF
#define DP0 0
#define DP1 1
#define DP2 2
#define DP3 3
#define DP4 4
#define DP5 5
#define DP6 6
#define DP7 7

#include "Arduino.h"

#ifdef F
#undef F
#endif

#define F(str)  str
#define __FlashStringHelper   char

#else

// Standard headers for AVR
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "Arduino.h"


#ifdef __cplusplus
extern "C" {
#endif
// Definitions specific for selected board
#if defined(__AVR_ATmega2560__)
#include "../board/mega/pins2_arduino.h"
#else
#if defined(__AVR_ATmega328P__)
#include "pins2_arduino.h"
#else
#if defined(__AVR_ATmega32U4__)
#include "../board/leonardo/pins2_arduino.h"
#else
#include "pins2_arduino.h"
#endif
#endif
#endif


// ===========================================================================
// Prototypes of digital I/O functions
// ===========================================================================
// Note: In standard Arduino these functions are implemented in
// hardware\arduino\cores\arduino\wiring_digital.c


// Arduino compatible functions (slower, but take pin as an integer)
// The pin is simple integer ( 0 thru 19).
// Note: these functions are never compiled into single instruction, even with cont pin.
static inline void pinMode2(uint8_t, uint8_t);
static inline void digitalWrite2(uint8_t, uint8_t);
static inline uint8_t digitalRead2(uint8_t);

// New version of functions which take pin code (GPIO_pin_t) as input parameter
static inline void pinMode2f(GPIO_pin_t pin, uint8_t mode );
static inline uint8_t digitalRead2f(GPIO_pin_t pin);
static inline void digitalWrite2f(GPIO_pin_t pin, uint8_t value);

// ===========================================================================
//

// User configuration defined in main.cpp or default value in pins2_arduino.h
#if GPIO2_PREFER_SPEED
#define   GPIO2_USE_INLINE_FUNCTIONS  1
#else
#define   GPIO2_USE_INLINE_FUNCTIONS  0
#endif

// Note: ATOMIC_BLOCK is macro in AVR Libc, but it cannot be used in current Arduino verison
// (1.0.5-r2), because the compiler options are not set to support C99 standard.
// That is why there is our own version for now
#define GPIO2_ATOMIC_BEGIN    { uint8_t matom_oldSREG = SREG; cli();
#define GPIO2_ATOMIC_END      SREG = matom_oldSREG; __asm__ volatile ("" ::: "memory"); }
/*
  // Version which can be used if the program is compiled using std=c99
  #define GPIO2_ATOMIC_BEGIN    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  #define GPIO2_ATOMIC_END      }
*/

// ARDUINO2_IOREGS_ABOVEFF - is defined in pins2_arduino.h, independently for each
// Arduino variant.
// GPIO2_OPTIONAL_ATOMIC_BEGIN is used in digitalWrite for the code which may result in single
// instruction for AVR MCUs with I/O register address below 0xFF but may result in more
// instructions for AVR MCUs with some registers above this address.
// For parts of code which should be always atomic, use GPIO2_ATOMIC_BEGIN
#if GPIO2_IOREGS_ABOVEFF
#define GPIO2_OPTIONAL_ATOMIC_BEGIN   GPIO2_ATOMIC_BEGIN
#define GPIO2_OPTIONAL_ATOMIC_END   GPIO2_ATOMIC_END
#else
#define GPIO2_OPTIONAL_ATOMIC_BEGIN
#define GPIO2_OPTIONAL_ATOMIC_END
#endif


// Prototypes for internal use
void internal_pinMode2(GPIO_pin_t pin, uint8_t mode);   // implemented in digital2.c
uint8_t internal_digitalRead2(GPIO_pin_t pin);        // implemented in digital2.c
void internal_digitalWrite2(GPIO_pin_t pin, uint8_t value); // implemented in digital2.c
static inline GPIO_pin_t Arduino_to_GPIO_pin(uint8_t pin);

// ===========================================================================
// Implementation of digital I/O functions
// ===========================================================================
//
// pinMode2
//
static inline void pinMode2(uint8_t, uint8_t) __attribute__((always_inline, unused));
static inline
void pinMode2(uint8_t pin, uint8_t mode )
{
  pinMode2f(Arduino_to_GPIO_pin(pin), mode);
}

//
// digitalRead2
// Size: 58 B with pin as a variable; 34 B with pin as a constant.
//
static inline uint8_t digitalRead2(uint8_t) __attribute__((always_inline, unused));
static inline
uint8_t digitalRead2(uint8_t pin)
{
  return digitalRead2f(Arduino_to_GPIO_pin(pin));
}

//
// digitalWrite2
//
static inline void digitalWrite2(uint8_t, uint8_t) __attribute__((always_inline, unused));
static inline
void digitalWrite2(uint8_t pin, uint8_t value)
{
  digitalWrite2f(Arduino_to_GPIO_pin(pin), value);
}


// =======================================
// New, fast functions
// The pin is one of the values defined in GPIO_pin_t, e.g. DP1 for digital pin 1, etc.
// Note: these functions can be compiled into single instruction if called with const pin
// and value/mode.
// ========================================

//
// pinMode2
//
// Set the pin to input, input with pull-up resistor or output mode.
// Note: There may be applications which require fast switching between input and
// output mode, so it is good to offer fast version for pins known at compile-time
// (where the compiler will perform "constant-folding" and we can get single instruction)
//
// Size: 2 B (single instruction) with const pin
//    24 B with pin as a variable and inline code
//    94 B with pin as a variable and non-inline code
//  Note: the size for inline variant is probably much smaller than the called variant
//  because the compiler could evaluate the "mode" in compile-time and generate code
//  just for the mode used (e.g. output), while for called variant
//  it has to generate the code for all possible values of "mode".
//
static inline void pinMode2f(GPIO_pin_t, uint8_t ) __attribute__((always_inline, unused));
static inline
void pinMode2f(GPIO_pin_t pin, uint8_t mode )
{

  if (__builtin_constant_p(pin) && __builtin_constant_p(mode))
  {
    // GPIO2_OPTIONAL_ATOMIC_ expands to nothing when not needed (atmega328)
    // and to ATOMIC_BLOCK when needed (atmega2560)
    GPIO2_OPTIONAL_ATOMIC_BEGIN
    // fast version which results in single instruction on atmega328
    if ( mode == OUTPUT )
    {
      GPIO_DDR_REG(pin) |= GPIO_PIN_MASK(pin);
    }
    else
    {
      if ( mode == INPUT_PULLUP )
      {
        GPIO_DDR_REG(pin) &= ~GPIO_PIN_MASK(pin);
        GPIO_PORT_REG(pin) |= GPIO_PIN_MASK(pin);
      }
      else
      {
        // input mode without pull-up
        GPIO_DDR_REG(pin) &= ~GPIO_PIN_MASK(pin);
        GPIO_PORT_REG(pin) &= ~GPIO_PIN_MASK(pin);
      }
    }
    GPIO2_OPTIONAL_ATOMIC_END
  }
  else
  {
    // The pin or mode are not know at compile-time but the user wants
    // inline code (GPIO2_PREFER_SPEED = 1)
#if GPIO2_USE_INLINE_FUNCTIONS

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

#else
    // Call non-inline version
    internal_pinMode2(pin, mode);
#endif

  }
}

//
// digitalRead2f
//
// Read value from given digital pin.
// The pin must be set to input mode using pinMode2f.
//
// Size: 8 B with const pin.
//    30 B with pin as a variable and non-inline worker
//    32 B with pin as a variable and inline code
//
static inline uint8_t digitalRead2f(GPIO_pin_t) __attribute__((always_inline, unused));
static inline
uint8_t digitalRead2f(GPIO_pin_t pin)
{
  // There is no need to disable interrupts for reading pin value, so
  // we can use the same code both for pins known and unknown at compile-time.
  // Unless we would want to save little program memory and make the
  // version with non-const pin non-inline.
  if (__builtin_constant_p(pin) )
  {
    if ((GPIO_PIN_REG(pin) & GPIO_PIN_MASK(pin)) != 0)
      return HIGH;
    else
      return LOW;
  }
  else
  {

#if GPIO2_USE_INLINE_FUNCTIONS

    if ((GPIO_PIN_REG(pin) & GPIO_PIN_MASK(pin)) != 0)
      return HIGH;
    else
      return LOW;

#else
    return internal_digitalRead2(pin);
#endif

  }
}


//
// digitalWrite2f
//
// Set value of a given pin. The pin must be in output mode.
//
// Size: 2 B with const pin (single SBI instruction). (Atmega328)
//    42 B (including rjmp) with pin as a variable and non-inline worker
//    24 B with pin as a variable and inline code
//  Note: the size for inline variant is probably  smaller than the called variant
//  because the compiler can evaluate the "value" in compile-time and generate code
//  just for the value used (e.g. HIGH), while for called variant
//  it has to generate the code for both HIGH and LOW values.
//
static inline void digitalWrite2f(GPIO_pin_t, uint8_t) __attribute__((always_inline, unused));
static inline
void digitalWrite2f(GPIO_pin_t pin, uint8_t value)
{
  // If we know the pin number and value at compile-time, the compiler
  // will compute everything during compilation and resulting code will
  // be single instruction - which does not need disabled interrupts
  if (__builtin_constant_p(pin) && __builtin_constant_p(value))
  {
    // ARDUINO2_ATOMIC expands to nothing when not needed (atmega328)
    // and to ATOMIC_BLOCK when needed (atmega2560)
    GPIO2_OPTIONAL_ATOMIC_BEGIN
    if ( value == 0 )
      GPIO_PORT_REG(pin) &= ~GPIO_PIN_MASK(pin);
    else
      GPIO_PORT_REG(pin) |= GPIO_PIN_MASK(pin);
    GPIO2_OPTIONAL_ATOMIC_END
  }
  else
  {
    // If pin or value must be computed in runtime, resulting code
    // will always require disabled interrupts.
#if GPIO2_USE_INLINE_FUNCTIONS

    // NOTE: if you make change here, make it also in internal_digitalWrite2()!
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

#else
    internal_digitalWrite2(pin, value);
#endif
  }

  // NOTE: Version of the code with macros GPIO_PORT_REG etc. placed directly
  // in the code results in the same code after compilation as if we use local
  // variables to store the mask and register address.
}

// Macro which obtains pin code from Arduino-style pin number
// This is used internally by inline function Arduino_to_GPIO_pin, which adds range check.
#define   GPIO_GET_PINDEF(pin)  (GPIO_pin_t)pgm_read_word(gpio_pins_progmem + (pin))

//
// Arduino_to_GPIO_pin
//
// Convert Arduino-style pin number (0 thru N) to pin code used by the fast '2f'
// functions.
//
// Note: If we use big switch, the compiler can evaluate this in compile-time
// for const pin numbers, which results in single instruction digitalRead/Write.
// It cannot do so if we use an array.
// BUT the compiler will only do the single instruction version if the function
// is declared inline. Otherwise it will generate normal call even for const pin.
// BUT inline function will be inlined for all calls, making the program
// very big.
// The solution used e.g. in digitalWrite2f, with __builtin_constant_p(pin),
// does not seem nice here; we would need 2 copies of the same code, one directly
// in this function for inlining and one in helper, non-inline function.
// Ideal solution would be normal, non-inline function which the compiler would
// automatically inline (make into single instruction) for const input. Since
// this seem impossible to achieve for me now, I use the array version of code and
// inline function.
static inline GPIO_pin_t Arduino_to_GPIO_pin(uint8_t) __attribute__((always_inline, unused));
static inline
GPIO_pin_t Arduino_to_GPIO_pin(uint8_t pin)
{
  if ( pin < GPIO_PINS_NUMBER )
    return GPIO_GET_PINDEF(pin);
  else
    return DP_INVALID;
}

int GPIO_to_Arduino_pin(GPIO_pin_t inPin);





#ifdef __cplusplus
} // extern "C"
#endif
#endif
#endif /* DIO2_H_ */
