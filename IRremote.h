/*
  Copyright (c) 2014-2018 NicoHood
  See the readme for credit to other people.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

// Include guard
#pragma once


#include <Arduino.h> // pinMode()
#define THEISR IRAM_ATTR
// Software version
#define IRL_VERSION 202

#if defined(ARDUINO_ARCH_AVR) || defined(DMBS_ARCH_AVR8)
#include <util/atomic.h>
#elif defined(ARDUINO_ARCH_ESP8266) || defined(ESP8266)
// copied from https://github.com/wizard97/SimplyAtomic/blob/master/esp8266.h

#ifndef __STRINGIFY
#define __STRINGIFY(a) #a
#endif

#ifndef xt_rsil
#define xt_rsil(level) (__extension__({uint32_t state; __asm__ __volatile__("rsil %0," __STRINGIFY(level) : "=a" (state)); state;}))
#endif

#ifndef xt_wsr_ps
#define xt_wsr_ps(state)  __asm__ __volatile__("wsr %0,ps; isync" :: "a" (state) : "memory")
#endif

static __inline__ void SA_iRestore(const  uint32_t *__s)
{
  xt_wsr_ps(*__s);
}

// Note value can be 0-15, 0 = Enable all interrupts, 15 = no interrupts
#define SA_ATOMIC_RESTORESTATE uint32_t _sa_saved   \
  __attribute__((__cleanup__(SA_iRestore))) = xt_rsil(15)

#define ATOMIC_RESTORESTATE
#define ATOMIC_BLOCK(A) \
  for ( SA_ATOMIC_RESTORESTATE, _sa_done =  1;    \
        _sa_done; _sa_done = 0 )
#else
#error "This library supports only AVR and ESP8266 Boards."
#endif


// Include all protocol implementations

//==============================================================================
// IRL_Receive Class
//==============================================================================

template<class T>
class CIRL_Receive
{
  public:
    // Attach the interrupt so IR signals are detected
    inline bool begin(uint8_t pin);
    inline bool end(uint8_t pin);

  protected:
    // Interface that is required to be implemented
    //static inline void interrupt(void);
    //static constexpr uint8_t interruptMode = FALLING|RISING|CHANGE;
};

//==============================================================================
// CIRL_Receive Implementation
//==============================================================================

template<class T>
bool CIRL_Receive<T>::begin(uint8_t pin)
{
  // Get pin ready for reading.
  pinMode(pin, INPUT_PULLUP);

  // Try to attach PinInterrupt first
  if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(pin), T::interrupt, T::interruptMode);
    return true;
  }

  // Return an error if none of them work (pin has no Pin(Change)Interrupt)
  return false;
}

template<class T>
bool CIRL_Receive<T>::end(uint8_t pin)
{
  // Disable pullup.
  pinMode(pin, INPUT);

  // Try to detach PinInterrupt first
  if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT) {
    detachInterrupt(digitalPinToInterrupt(pin));
    return true;
  }


  // Return an error if none of them work (pin has no Pin(Change)Interrupt)
  return false;
}


//==============================================================================
// IRL_Time Class
//==============================================================================

template<class T>
class CIRL_Time
{
  public:
    // User API to access library data
    inline uint32_t timeout(void);
    inline uint32_t lastEvent(void);
    inline uint32_t nextEvent(void);

    // Interface that is required to be implemented
    //static constexpr uint32_t timespanEvent = VALUE;
    //static constexpr uint32_t limitTimeout = VALUE;

  protected:
    // Time mangement functions
    static inline uint16_t nextTime(void);

    // Time values for the last interrupt and the last valid protocol
    static uint32_t mlastTime;
    static volatile uint32_t mlastEvent;
};


//==============================================================================
// Static Data
//==============================================================================

// Protocol temporary data
template<class T> uint32_t CIRL_Time<T>::mlastTime = 0;
template<class T> volatile uint32_t CIRL_Time<T>::mlastEvent = 0;


//==============================================================================
// CIRL_Time Implementation
//==============================================================================

/*
   Returns duration between last interrupt and current time.
   This will safe the last interrupt time to the current time.
*/
template<class T>
uint16_t CIRL_Time<T>::nextTime(void) {
  // Save the duration between the last reading
  uint32_t time = micros();
  uint32_t duration_32 = time - mlastTime;
  mlastTime = time;

  // Calculate 16 bit duration. On overflow sets duration to a clear timeout
  uint16_t duration = duration_32;
  if (duration_32 > 0xFFFF) {
    duration = 0xFFFF;
  }

  return duration;
}


/*
   Return relativ time between last event time (in micros)
*/
template<class T>
uint32_t CIRL_Time<T>::timeout(void)
{
  uint32_t timeout;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    timeout = mlastEvent;
  }

  uint32_t time = micros();
  timeout = time - timeout;

  return timeout;
}


/*
   Return absolute last event time (in micros)
*/
template<class T>
uint32_t CIRL_Time<T>::lastEvent(void)
{
  uint32_t time;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    time = mlastEvent;
  }

  return time;
}


/*
   Return when the next event can be expected.
   Zero means at any time.
   Attention! This value is a little bit too high in general.
   Also for the first press it is even higher than it should.
*/
template<class T>
uint32_t CIRL_Time<T>::nextEvent(void)
{
  auto time = timeout();
  auto timespan = static_cast<T*>(this)->timespanEvent;

  if (time >= timespan) {
    return 0;
  }

  return timespan - time;
}


template<class T, class Protocol_data_t>
class CIRL_Protocol
{
  public:
    // User API to access library data
    Protocol_data_t read(void);

  protected:
    // Interface that is required to be implemented
    //inline Nec_data_t getData(void);
    //inline void resetReading(void);
};


//==============================================================================
// CIRL_Protocol Implementation
//==============================================================================

template<class T, class Protocol_data_t>
THEISR Protocol_data_t CIRL_Protocol<T, Protocol_data_t>::read(void)
{
  // If nothing was received return an empty struct
  Protocol_data_t retdata = Protocol_data_t();

  // Disable interrupts while accessing volatile data
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Check and get data if we have new.
    if (static_cast<T*>(this)->available())
    {
      // Set last ISR to current time.
      // This is required to not trigger a timeout afterwards
      // and read corrupted data. This might happen
      // if the reading loop is too slow.
      static_cast<T*>(this)->mlastTime = micros();

      // Save the protocol data
      retdata = static_cast<T*>(this)->getData();

      // Reset reading
      static_cast<T*>(this)->resetReading();
    }
  }

  // Return the new protocol information to the user
  return retdata;
}


//==============================================================================
// CIRL_DecodeSpaces Class
//==============================================================================

template<class T, int blocks>
class CIRL_DecodeSpaces
{
  public:
    // User API to access library data
    inline bool available(void);
    inline bool receiving(void);

  protected:
    // Temporary buffer to hold bytes for decoding the protocol
    static volatile uint8_t count;
    static uint8_t data[blocks];

    // Interrupt function that is attached
    inline void resetReading(void);
    static void interrupt(void);
    static constexpr uint8_t interruptMode = FALLING;

    // Interface that is required to be implemented
    //static inline bool checksum(void);
    //static inline void holding(void);
    //static constexpr uint32_t limitTimeout = VALUE;
    //static constexpr uint32_t limitLead = VALUE;
    //static constexpr uint32_t limitHolding = VALUE;
    //static constexpr uint32_t limitLogic = VALUE;
    //static constexpr uint32_t limitRepeat = VALUE;
    //static constexpr uint8_t irLength = VALUE;
};


//==============================================================================
// Static Data
//==============================================================================

// Protocol temporary data
template<class T, int blocks>
volatile uint8_t CIRL_DecodeSpaces<T, blocks>::count = 0;
template<class T, int blocks>
uint8_t CIRL_DecodeSpaces<T, blocks>::data[blocks] = { 0 };


//==============================================================================
// CIRL_DecodeSpaces Implementation
//==============================================================================

template<class T, int blocks>
bool CIRL_DecodeSpaces<T, blocks>::available(void) {
  return count > (T::irLength / 2);
}


template<class T, int blocks>
void CIRL_DecodeSpaces<T, blocks>::resetReading(void) {
  // Reset reading
  count = 0;
}


template<class T, int blocks>
THEISR void CIRL_DecodeSpaces<T, blocks>::interrupt(void)
{
  // Block if the protocol is already recognized
  if (count > (T::irLength / 2)) {
    return;
  }

  // Get time between previous call and decode
  auto duration = T::nextTime();

  // On a timeout abort pending readings and start next possible reading
  if (duration >= T::limitTimeout) {
    count = 0;
  }

  // On a reset (error in decoding) wait for a timeout to start a new reading
  // This is to not conflict with other protocols while they are sending 0/1
  // which might be similar to a lead in this protocol
  else if (count == 0) {
    return;
  }

  // Check Mark Lead (requires a timeout)
  else if (count == 1)
  {
    // Wrong lead
    if (duration < T::limitHolding)
    {
      count = 0;
      return;
    }
    // Check for a "button holding" lead
    else if (duration < T::limitLead)
    {
      // Abort if last valid button press is too long ago
      if ((T::mlastTime - \
           T::mlastEvent) >= \
          T::limitRepeat)
      {
        count = 0;
        return;
      }

      // Received a Nec Repeat signal
      // Next mark (stop bit) ignored due to detecting techniques
      T::holding();
      count = (T::irLength / 2);
      T::mlastEvent = T::mlastTime;
    }
    // Else normal lead, continue processing
  }

  // Check different logical space pulses (mark + space)
  else
  {
    // Get number of the Bits (starting from zero)
    // Substract the first lead pulse
    uint8_t length = count - 2;

    // Move bits (MSB is zero)
    data[length / 8] >>= 1;

    // Set MSB if it's a logical one
    if (duration >= T::limitLogic) {
      data[length / 8] |= 0x80;
    }

    // Last bit (stop bit following)
    if (count >= (T::irLength / 2))
    {
      // Check if the protcol's command checksum is correct
      if (T::checksum()) {
        T::mlastEvent = T::mlastTime;
      }
      else {
        count = 0;
        return;
      }
    }
  }

  // Next reading, no errors
  count++;
}

/*
   Return true if we are currently receiving new data
*/
template<class T, int blocks>
bool CIRL_DecodeSpaces<T, blocks>::receiving(void)
{
  bool ret = false;

  // Provess with interrupts disabled to avoid any conflicts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Check if we already recognized a timed out
    if (count == 0) {
      ret = false;
    }
    else
    {
      // Calculate difference between last interrupt and now
      uint32_t timeout = T::mlastTime;
      uint32_t time = micros();
      timeout = time - timeout;

      // Check for a new timeout
      if (timeout >= T::limitTimeout) {
        count = 0;
        ret = false;
      }
      // We are currently receiving
      else {
        ret = true;
      }
    }
  }

  return ret;
}


//==============================================================================
// Protocol Definitions
//==============================================================================

// NEC
// IRP notation:
// {38.4k,564}<1,-1|1,-3>(16,-8,D:8,S:8,F:8,~F:8,1,-78,(16,-4,1,-173)*)
// Lead + Space logic
#define NEC_HZ                38000UL
#define NEC_PULSE             564UL
#define NEC_ADDRESS_LENGTH    16
#define NEC_COMMAND_LENGTH    16
#define NEC_DATA_LENGTH       (NEC_ADDRESS_LENGTH + NEC_COMMAND_LENGTH)
#define NEC_BLOCKS            (NEC_DATA_LENGTH / 8)
// 2 for lead + space, each block has mark and space
#define NEC_LENGTH            (2 + NEC_DATA_LENGTH * 2)
#define NEC_TIMEOUT           (NEC_PULSE * 78UL)
#define NEC_TIMEOUT_HOLDING   (NEC_PULSE * 173UL)
#define NEC_TIMESPAN_HOLDING  (NEC_TIMEOUT_HOLDING + NEC_LOGICAL_HOLDING)
#define NEC_MARK_LEAD         (NEC_PULSE * 16UL)
#define NEC_MARK_HOLDING      (NEC_PULSE * 16UL)
#define NEC_SPACE_LEAD        (NEC_PULSE * 8UL)
#define NEC_SPACE_HOLDING     (NEC_PULSE * 4UL)
#define NEC_LOGICAL_LEAD      (NEC_MARK_LEAD + NEC_SPACE_LEAD)
#define NEC_LOGICAL_HOLDING   (NEC_MARK_LEAD + NEC_SPACE_HOLDING)
#define NEC_MARK_ZERO         (NEC_PULSE * 1UL)
#define NEC_MARK_ONE          (NEC_PULSE * 1UL)
#define NEC_SPACE_ZERO        (NEC_PULSE * 1UL)
#define NEC_SPACE_ONE         (NEC_PULSE * 3UL)
#define NEC_LOGICAL_ZERO      (NEC_MARK_ZERO + NEC_SPACE_ZERO)
#define NEC_LOGICAL_ONE       (NEC_MARK_ONE + NEC_SPACE_ONE)

// Decoding limits
#define NEC_LIMIT_LOGIC       ((NEC_LOGICAL_ONE + NEC_LOGICAL_ZERO) / 2)
#define NEC_LIMIT_HOLDING     ((NEC_LOGICAL_HOLDING + NEC_LOGICAL_ONE) / 2)
#define NEC_LIMIT_LEAD        ((NEC_LOGICAL_LEAD + NEC_LOGICAL_HOLDING) / 2)
#define NEC_LIMIT_TIMEOUT     ((NEC_TIMEOUT + NEC_LOGICAL_LEAD) / 2)
#define NEC_LIMIT_REPEAT      (NEC_TIMESPAN_HOLDING * 3 / 2)

/*
   Nec pulse demonstration:

  ---|                |--------| |---| |-|   ... -| |----------/ ~ /----------|
     |                |        | |   | | |   ...  | |                         |
     |                |        | |   | | |   ...  | |                         |
     |----------------|        |-|   |-| |-  ...  |-|                         |
     |          Lead           |Log 1|Lg0|  Data  |E|         Timeout         |-

  ---|                |----| |---------------------/ ~ /----------------------|
     |                |    | |                                                |
     |                |    | |                                                |
     |----------------|    |-|                                                |
     |      Holding        |E|                Timeout Holding                 |-
     |                            Timespan Holding                            |

    2 Pulses: |-+-| Logical 0
    3 Pulses: |----| limitLogic
    4 Pulses: |-+---| Logical 1
   12 Pulses: |-------------| limitHolding
   20 Pulses: |----------------+----| Holding
   22 Pulses: |-----------------------| limitLead
   24 Pulses: |----------------+--------| Lead
   51 Pulses: |-----------------/ ~ /-----------------| limitTimeout
   78 Pulses: |-------------------------/ ~ /-------------------------| Timeout
*/

typedef uint16_t Nec_address_t;
typedef uint8_t Nec_command_t;

// Struct that is returned by the read() function
struct Nec_data_t
{
  Nec_address_t address;
  Nec_command_t command;
};

//==============================================================================
// Nec Decoding Class
//==============================================================================

class CNec : public CIRL_Receive<CNec>,
  public CIRL_Time<CNec>,
  public CIRL_Protocol<CNec, Nec_data_t>,
  public CIRL_DecodeSpaces<CNec, NEC_BLOCKS>
{
  protected:
    static constexpr uint32_t timespanEvent = NEC_TIMESPAN_HOLDING;
    static constexpr uint32_t limitTimeout = NEC_LIMIT_TIMEOUT;
    static constexpr uint32_t limitLead = NEC_LIMIT_LEAD;
    static constexpr uint32_t limitHolding = NEC_LIMIT_HOLDING;
    static constexpr uint32_t limitLogic = NEC_LIMIT_LOGIC;
    static constexpr uint32_t limitRepeat = NEC_LIMIT_REPEAT;
    static constexpr uint8_t irLength = NEC_LENGTH;

    friend CIRL_Receive<CNec>;
    friend CIRL_Protocol<CNec, Nec_data_t>;
    friend CIRL_DecodeSpaces<CNec, NEC_BLOCKS>;

    // Protocol interface functions
    inline Nec_data_t getData(void);
    static inline bool checksum(void);
    static inline void holding(void);
};


//==============================================================================
// Nec Decoding Implementation
//==============================================================================

Nec_data_t CNec::getData(void) {
  Nec_data_t retdata;
  retdata.address = ((uint16_t)data[1] << 8) | ((uint16_t)data[0]);
  retdata.command = data[2];
  return retdata;
}


bool CNec::checksum(void) {
  return uint8_t((data[2] ^ (~data[3]))) == 0x00;
}


void CNec::holding(void) {
  // Flag repeat signal via "invalid" address and empty command
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0x00;
}
//==============================================================================
// API Class
//==============================================================================

typedef void(*NecEventCallback)(void);
#define NEC_API_PRESS_TIMEOUT (500UL * 1000UL) // 0.5 seconds

template<const NecEventCallback callback, const uint16_t address = 0x0000>
class CNecAPI : public CNec
{
  public:
    // User API to access library data
    inline void read(void);
    inline uint8_t command(void);
    inline uint8_t count(void);
    inline uint8_t duration(bool raw = false);
    inline uint8_t released(bool samebutton = false);
    inline constexpr uint32_t getTimeout(void);
    inline uint32_t nextTimeout(void);

  protected:
    // Differenciate between timeout types
    enum TimeoutType : uint8_t
    {
      NO_TIMEOUT,     // Keydown
      TIMEOUT,         // Key release with timeout
      NEXT_BUTTON,     // Key release, pressed again
      NEW_BUTTON,     // Key release, another key is pressed
    } NecTimeoutType;

    // Keep track which key was pressed/held down how often
    uint8_t lastCommand = 0;
    uint8_t lastPressCount = 0;
    uint8_t lastHoldCount = 0;
};

//==============================================================================
// API Class Implementation
//==============================================================================

// Reads data from the nec protocol (if available) and processes it.
template<const NecEventCallback callback, const uint16_t address>
void CNecAPI<callback, address>::read(void) {
  auto data = CNec::read();

  // Check if the correct protocol and address (optional) is used
  bool firstCommand = data.address != 0xFFFF;
  if ((data.address == 0) || (address && firstCommand && (data.address != address)))
  {
    // Call the remote function again once the keypress timed out
    if (lastPressCount && (timeout() > getTimeout()))
    {
      // Flag timeout event, key was released and the current chain is over
      NecTimeoutType = TIMEOUT;
      callback();

      // Reset the button press and hold count after a timeout
      lastPressCount = 0;
      lastHoldCount = 0;
    }
    return;
  }

  // Count the first button press
  if (firstCommand)
  {
    // The same button was pressed twice in a short timespawn (500ms)
    if (data.command == lastCommand)
    {
      // Flag that the last button hold is over, the same key is held down again
      if (lastPressCount) {
        NecTimeoutType = NEXT_BUTTON;
        callback();
      }

      // Increase pressing streak
      if (lastPressCount < 255) {
        lastPressCount++;
      }
    }
    // Different button than before
    else
    {
      // Flag that the last button hold is over, a differnt key is now held down
      if (lastPressCount) {
        NecTimeoutType = NEW_BUTTON;
        callback();
      }
      lastPressCount = 1;
    }

    // Start a new series of button holding
    lastHoldCount = 0;

    // Save the new command. On a repeat (below) don't safe it.
    lastCommand = data.command;
  }
  // Count the button holding
  else
  {
    // Abort if no first press was recognized (after reset)
    if (!lastPressCount) {
      return;
    }

    // Increment holding count
    if (lastHoldCount < 255) {
      lastHoldCount++;
    }
  }

  // Call the remote function and flag that the event was just received
  NecTimeoutType = NO_TIMEOUT;
  callback();
}


template<const NecEventCallback callback, const uint16_t address>
uint8_t CNecAPI<callback, address>::command(void)
{
  return lastCommand;
}

// Number of repeating button presses in a row
template<const NecEventCallback callback, const uint16_t address>
uint8_t CNecAPI<callback, address>::count(void)
{
  return lastPressCount;
}

// Duration (count) how long the current button press was held down.
// Pass true to also recognize keyup events
template<const NecEventCallback callback, const uint16_t address>
uint8_t CNecAPI<callback, address>::duration(bool raw)
{
  // Only recognize the actual keydown event
  if (NecTimeoutType == NO_TIMEOUT || raw) // TODO reorder?
  {
    return 1 + lastHoldCount;
  }
  return 0;
}

// True when the button is released:
// 1. Timeout of press series
// 2. Next press, new button
// 3. Next press, same button
// `--> pass true as parameter,
//      useful when measuring idential or single button press durations
// False when the button is held down:
// 1. Initial button press
// 2. Holding button down
// 3. Renewed button press
// Usually you want to use timeout() to check if the series ends
template<const NecEventCallback callback, const uint16_t address>
uint8_t CNecAPI<callback, address>::released(bool samebutton)
{
  if (NecTimeoutType == TIMEOUT || NecTimeoutType == NEW_BUTTON) {
    return 1 + lastHoldCount;
  }
  if (samebutton && NecTimeoutType == NEXT_BUTTON) {
    return 1 + lastHoldCount;
  }
  return 0;
}

template<const NecEventCallback callback, const uint16_t address>
constexpr uint32_t CNecAPI<callback, address>::getTimeout(void) {
  return NEC_API_PRESS_TIMEOUT;
}

// Return when the next timeout triggers.
// Zero means it already timed out.
template<const NecEventCallback callback, const uint16_t address>
uint32_t CNecAPI<callback, address>::nextTimeout(void)
{
  auto time = timeout();
  auto timeout = getTimeout();

  if (time >= timeout) {
    return 0;
  }

  return timeout - time;
}

//==============================================================================
// Protocol Definitions
//==============================================================================

//HashIR
#define HASHIR_BLOCKS 255        // 0-65535 (maximum input length)
#define HASHIR_TIMEOUT (0xFFFF/4)   // 65535, max timeout
#define HashIR_TIMESPAN (HASHIR_TIMEOUT * 3)
#define HASHIR_TIME_THRESHOLD 10000UL // 0-32bit

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619UL
#define FNV_BASIS_32 2166136261UL

typedef uint8_t HashIR_address_t;
typedef uint32_t HashIR_command_t;

// Struct that is returned by the read() function
struct HashIR_data_t
{
  HashIR_address_t address;
  HashIR_command_t command;
};

//==============================================================================
// Hash Decoding Class
//==============================================================================

class CHashIR : public CIRL_Receive<CHashIR>,
  public CIRL_Time<CHashIR>,
  public CIRL_Protocol<CHashIR, HashIR_data_t>
{
  public:
    // User API to access library data
    inline bool available(void);
    inline bool receiving(void);

  protected:
    static constexpr uint32_t timespanEvent = HashIR_TIMESPAN;

    friend CIRL_Receive<CHashIR>;
    friend CIRL_Protocol<CHashIR, HashIR_data_t>;

    // Interrupt function that is attached
    inline void resetReading(void);
    static inline void interrupt(void);
    static constexpr uint8_t interruptMode = CHANGE;

    // Protocol interface functions
    inline HashIR_data_t getData(void);
    static inline bool checksum(void);
    static inline void holding(void);

    // Protocol variables
    static volatile uint8_t count;
    static uint32_t hash;
    static volatile uint16_t lastDuration;
};


//==============================================================================
// Hash Decoding Implementation
//==============================================================================

HashIR_data_t CHashIR::getData(void) {
  // Save address as length.
  // You can check the address/length to prevent triggering on noise
  HashIR_data_t retdata;
  retdata.address = count;
  retdata.command = hash;
  return retdata;
}


bool CHashIR::available(void) {
  // First look for a timeout
  receiving();
  bool ret;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    ret = lastDuration == 0;
  }
  return ret;
}


void CHashIR::resetReading(void) {
  // Reset reading
  hash = FNV_BASIS_32;
  lastDuration = 0xFFFF;
  count = 0;
}


bool CHashIR::receiving(void)
{
  bool ret = false;

  // Provess with interrupts disabled to avoid any conflicts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    // Check if we already recognized a timed out
    if (count == 0) {
      ret = false;
    }
    else
    {
      // Calculate difference between last interrupt and now
      uint32_t timeout = mlastTime;
      uint32_t time = micros();
      timeout = time - timeout;

      // Check for a new timeout
      if (timeout >= HASHIR_TIMEOUT)
      {
        // Flag new data if we previously received data
        if (count > 1) {
          count--;
          lastDuration = 0;
          mlastEvent = mlastTime;
        }
        else {
          count = 0;
        }
        ret = false;
      }
      // We are currently receiving
      else {
        ret = true;
      }
    }
  }

  return ret;
}

THEISR void CHashIR::interrupt(void)
{
  // Block if the protocol is already recognized
  if (lastDuration == 0) {
    return;
  }

  // Get time between previous call and decode
  auto duration = nextTime();

  // Reading timed out
  if (duration >= HASHIR_TIMEOUT)
  {
    // Start a new reading sequence.
    if (count == 0) {
      count++;
    }
    // Ignore the very first timeout of each reading.
    // Otherwise flag a new input and stop reading.
    else if (count != 1) {
      count--;
      lastDuration = 0;
      mlastEvent = mlastTime;
    }
    return;
  }

  // Only save data if a sequence is running.
  // This is required to avoid corrupted data
  // when starting capturing at the middle of a sequence.
  if (count)
  {
    // Converts the raw code values into a 32-bit hash code.
    // Hopefully this code is unique for each button.
    // This isn't a "real" decoding, just an arbitrary value.
    // Code taken from https://github.com/z3t0/Arduino-IRremote

    // Only compare after the first value got received
    if (count > 1)
    {
      // Get both values
      auto oldval = lastDuration;
      auto newval = duration;

      // Compare two tick values, returning 0 if newval is shorter,
      // 1 if newval is equal, and 2 if newval is longer
      // Use a tolerance of 75%
      uint8_t value = 1;
      if (newval < (oldval * 3 / 4)) {
        value = 0;
      }
      else if (oldval < (newval * 3 / 4)) {
        value = 2;
      }

      // Add value into the hash
      hash = (hash * FNV_PRIME_32) ^ value;
    }

    // Save last time and count up
    count++;

    // Flag a new input if buffer is full
    if (count >= HASHIR_BLOCKS) {
      lastDuration = 0;
      mlastEvent = mlastTime;
    }
    else {
      lastDuration = duration;
    }
  }
}
