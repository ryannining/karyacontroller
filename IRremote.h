#pragma once
#include <Arduino.h>


// NEC protocol parameters (mostly unchanged)
#define NEC_HZ                38000UL
#define NEC_PULSE             564UL
#define NEC_ADDRESS_LENGTH    16
#define NEC_COMMAND_LENGTH    16
#define NEC_DATA_LENGTH       (NEC_ADDRESS_LENGTH + NEC_COMMAND_LENGTH)
#define NEC_BLOCKS            (NEC_DATA_LENGTH / 8)
#define NEC_TIMEOUT           (NEC_PULSE * 110UL)  // Increased timeout
#define NEC_MARK_LEAD         (NEC_PULSE * 16UL)
#define NEC_SPACE_LEAD        (NEC_PULSE * 8UL)
#define NEC_MARK_ZERO         (NEC_PULSE * 1UL)
#define NEC_MARK_ONE          (NEC_PULSE * 1UL)
#define NEC_SPACE_ZERO        (NEC_PULSE * 1UL)
#define NEC_SPACE_ONE         (NEC_PULSE * 3UL)
#define NEC_LOGICAL_ZERO      (NEC_MARK_ZERO + NEC_SPACE_ZERO)
#define NEC_LOGICAL_ONE       (NEC_MARK_ONE + NEC_SPACE_ONE)

typedef uint16_t Nec_address_t;
typedef uint8_t Nec_command_t;

struct Nec_data_t {
  Nec_address_t address;
  Nec_command_t command;
};

// Function Prototypes

// Internal Variables
static volatile uint8_t ir_count = 0;
static volatile uint32_t ir_data = 0;
static volatile uint32_t ir_lastTime = 0;
static volatile bool ir_complete = false;

// Callback function for interrupt
static IRAM_ATTR void IR_IRQHandler();


static IRAM_ATTR void IR_IRQHandler() {
  uint32_t currentTime = micros();
  uint32_t duration = currentTime - ir_lastTime;
  ir_lastTime = currentTime;



  // Reset if we get a long pause
  if (duration > 15000) {
    ir_count = 0;
    ir_data = 0;
    return;
  }

  // Look for the start of transmission
  if (ir_count == 0) {
    if (duration > 12000 && duration < 15000) {  // Adjusted for observed lead pulse
      ir_count = 1;
    }
    return;
  }

  // Skip the space after the lead pulse
  if (ir_count == 1) {
    ir_count++;
    return;
  }

  // Process the rest of the signal
  if (ir_count < 33) {  // 32 bits + 1 stop bit
    ir_data >>= 1;
    if (duration > 1700) {  // Adjusted threshold for logical '1'
      ir_data |= (1<<31);
    }
    ir_count++;
  }

  if (ir_count >= 33) {
    ir_complete = true;
    ir_count = 0;  // Reset for next transmission
  }
}


static bool IRBegin(uint8_t pin) {
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), IR_IRQHandler, FALLING);
  return true;
}

static void IREnd(uint8_t pin) {
  detachInterrupt(digitalPinToInterrupt(pin));  // Using D1 as defined in setup
}

static bool IRAvailable() {
  return ir_complete;
}

static Nec_data_t IRRead() {
  Nec_data_t data = {0, 0};
  
  if (ir_complete) {
    data.address = (ir_data) & 0xFF;
    data.command = (ir_data >> 16) & 0xFF;
    ir_complete = false;
    ir_count = 0;
    ir_data = 0;
  }
  
  return data;
}
