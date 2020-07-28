#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include <stdint.h>
#include "config_pins.h"
#include "common.h"
#include "platform.h"

#ifdef servo_pin
uint32_t servo_timer = 0;
int32_t servo_pos = 0;
#ifdef __AVR__
static volatile int ISRCount, remainder, counter; // iteration counter used in the interrupt routines;

ISR (TIMER2_OVF_vect)
{
  ++ISRCount; // increment the overlflow counter
  if (ISRCount ==  counter) { // are we on the final iteration for this channel
    TCNT2 =  remainder;   // yes, set count for overflow after remainder ticks
  }
  if (ISRCount ==  (counter + 1)) {
    digitalWrite( servo_pin, LOW  ); // pulse this channel low if active
  } else if (ISRCount >  157) {
    digitalWrite( servo_pin, HIGH); // pulse this channel low if active
    ISRCount = 0; // reset the isr iteration counter
    TCNT2 = 0;    // reset the clock counter register
  }
}
#endif

void servo_set(int us)
{
  counter = us / 128;
  remainder = 255 - (2 * (us - ( counter * 128)));
}

void servo_init()
{
  pinMode(servo_pin, OUTPUT);

#ifdef __AVR__
  servo_set(1000);

  ISRCount = 0;  // clear the value of the ISR counter;

  /* setup for timer 2 */
  TIMSK2 = 0;  // disable interrupts
  TCCR2A = 0;  // normal counting mode
  TCCR2B = _BV(CS21); // set prescaler of 8 = 2MHZ
  TCNT2 = 0;     // clear the timer2 count
  TIFR2 = _BV(TOV2);  // clear pending interrupts;
  TIMSK2 =  _BV(TOIE2) ; // enable the overflow interrupt
#endif
}

void servo_loop()
{

#ifdef __AVR__
#else
  uint32_t m = micros();
  if (m - servo_timer > 20000) {
    servo_timer = m;
    digitalWrite(servo_pin, HIGH);
  }
  if (m - servo_timer > servo_pos) {
    digitalWrite(servo_pin, LOW);
  }
#endif
}
#else
void servo_loop() {}
void servo_init() {}
void servo_set(int us) {}

#endif


int somedelay(int32_t n)
{
  float f = 0;
  int m = 10;

  while (m--) {
    int nn = n;
    while (nn--) {
      f += n;
      asm("");
    }
  }
  return f + n;
}

//#define somedelay(n) delayMicroseconds(n);
int dogfeed = 0;

#ifndef ISPC
#include<Arduino.h>
#define dogfeedevery 200 // loop
// ESP8266 specific code here
#else
#define dogfeedevery 100000 // loop
#include<sys/time.h>
uint32_t micros()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return 1000000 * tv.tv_sec + tv.tv_usec;
}

#endif

int feedthedog()
{
  if (dogfeed++ > dogfeedevery) {
    dogfeed = 0;
#if defined(__AVR__)
    // AVR specific code here
#elif defined(ESP8266)
    // ESP8266 specific code here
    ESP.wdtFeed();
#elif defined(ESP32)
//  esp_task_wdt_reset();
#else
#endif
    //zprintf(PSTR("Feed the dog\n"));
    return 1;
  }
  return 0;
}


// ======================== TIMER for motionloop =============================
#define timerPause()
#define timerResume()

uint32_t	next_step_time;
#ifdef USETIMER1
int busy1 = 0;
volatile uint32_t ndelay = 0, ndelay2 = 0;

// ======= the same code for all cpu ======
inline int THEISR timercode() {
  if (ndelay2)
  {
    // turn off laser , laser constant burn mode
    ndelay = ndelay2;
    ndelay2 = 0;
    LASER(!LASERON);
  } else {
    if (ndelay < 30000) {
      ndelay = 30;
      coreloopm();
#ifdef __AVR__
      TCNT1 = 0;
#endif
    } else {
      ndelay -= 30000;
    }
  }
  return ndelay >= 30000 ? 30000 : ndelay;
}

// ==============================================
/*
    AVR TIMER
*/
#ifdef __AVR__
#define USETIMEROK
#define MINDELAY 5000


ISR(TIMER1_COMPA_vect)
//void tm()
{
  TIMSK1 &= ~(1 << OCIE1A);

  // stepper tick
  //cli();
  int d = timercode();
  if (d < 6)d = 6;
  OCR1A = d;
  //sei();
  TIMSK1 |= (1 << OCIE1A);
  //if (ndelay>40)zprintf(PSTR("%d\n"),fi(ndelay));
}

void timer_init()
{
  TCCR1A = 0;  // Steup timer 1 interrupt to no prescale CTC mode
  TIMSK1 = 0;
  TCNT1 = 0;
  TCCR1B = (1 << CS11); // no prescaler == 0.0625 usec tick | 001 = clk/1
  OCR1A  = 65500; //start off with a slow frequency.
  TIMSK1 |= (1 << OCIE1A); // Enable interrupt
}

#endif // avr timer

/*
    =======================================  ARM TIMER   =======================================
*/
#ifdef __ARM__
//#include "HardwareTimer.h"
#define USETIMEROK
//HardwareTimer timer1(2);
#define MINDELAY 5000

void tm()
{
  //Timer1.pause();
  //zprintf(PSTR(".\n"));
  int d = timercode();
  if (d < 6)d = 6;

  Timer1.setOverflow(d);
  Timer1.setCompare1(d);
  Timer1.resume();
}

void timer_init()
{
  /*
    Timer2.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer2.setPeriod(LED_RATE); // in microseconds
    Timer2.setCompare(TIMER_CH1, 1);      // overflow might be small
    Timer2.attachInterrupt(TIMER_CH1, handler_led);
  */
  Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer1.setPrescaleFactor(9);
  Timer1.setOverflow(1000); // in microseconds
  Timer1.setCompare(TIMER_CH1, 3000);      // overflow might be small
  Timer1.attachInterrupt(TIMER_CH1, tm);
}

#endif // arm
// -------------------------------------   ESP8266  ----------------------------------------------------------
#ifdef ESP8266
#define USETIMEROK
#define MINDELAY 5000
#define usetmr1


void ICACHE_RAM_ATTR tm()
{
  noInterrupts();
  int d = timercode();

#ifdef usetmr1
  timer1_write(d<6?6:d);
#else
  timer0_write(ESP.getCycleCount() + 16 * (d<6?6:d));
#endif
  interrupts();
}

void timer_init()
{
  //Initialize Ticker every 0.5s
  noInterrupts();
#ifdef usetmr1
  timer1_isr_init();
  timer1_attachInterrupt(tm);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(1000); //120000 us
#else
  timer0_isr_init();
  timer0_attachInterrupt(tm);
  timer0_write(ESP.getCycleCount() + 16 * 1000); //120000 us
#endif
  interrupts();
}

#endif // esp8266
#ifdef ESP32
#define USETIMEROK
typedef struct {
    union {
        struct {
            uint32_t reserved0:   10;
            uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
            uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
            uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
            uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
            uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
            uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
            uint32_t enable:       1;             /*When set  timer 0/1 time-base counter is enabled*/
        };
        uint32_t val;
    } config;
    uint32_t cnt_low;                             /*Register to store timer 0/1 time-base counter current value lower 32 bits.*/
    uint32_t cnt_high;                            /*Register to store timer 0 time-base counter current value higher 32 bits.*/
    uint32_t update;                              /*Write any value will trigger a timer 0 time-base counter value update (timer 0 current value will be stored in registers above)*/
    uint32_t alarm_low;                           /*Timer 0 time-base counter value lower 32 bits that will trigger the alarm*/
    uint32_t alarm_high;                          /*Timer 0 time-base counter value higher 32 bits that will trigger the alarm*/
    uint32_t load_low;                            /*Lower 32 bits of the value that will load into timer 0 time-base counter*/
    uint32_t load_high;                           /*higher 32 bits of the value that will load into timer 0 time-base counter*/
    uint32_t reload;                              /*Write any value will trigger timer 0 time-base counter reload*/
} xhw_timer_reg_t;

typedef struct xhw_timer_s {
        xhw_timer_reg_t * dev;
        uint8_t num;
        uint8_t group;
        uint8_t timer;
        portMUX_TYPE lock;
} xhw_timer_t;


hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


void THEISR xtimerAlarmWrite(xhw_timer_s *timer, uint64_t alarm_value){
    timer->dev->alarm_high = (uint32_t) (alarm_value >> 32);
    timer->dev->alarm_low = (uint32_t) alarm_value;
    timer->dev->config.autoreload = true;
    timer->dev->config.alarm_en = 1;
}

void THEISR tm() {
  portENTER_CRITICAL_ISR(&timerMux);
  int d = timercode();

  xtimerAlarmWrite((xhw_timer_s*)timer, d<6?6:d);
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void timer_init()
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &tm, true);
  xtimerAlarmWrite((xhw_timer_s*)timer, 12000);
  //zprintf(PSTR("Time init\n"));
}

/*
#undef timerPause()
#undef timerResume()
int tmrstack=0;
void THEISR timerPause(){
  tmrstack++;
  if (tmrstack==1)((xhw_timer_s*) timer)->dev->config.alarm_en = 0; 
}
void THEISR timerResume(){
  tmrstack--;
  if (tmrstack==0)((xhw_timer_s*) timer)->dev->config.alarm_en = 1;
}
*/

#endif //esp32

// Laser constant burn timer
void THEISR timer_set2(int32_t delay, int32_t delayL)
{
  if (delayL) {
    if (delayL > delay -6)delay=delayL+6;
    ndelay2 = delay - delayL; // the rest delay after laser on
    ndelay = delayL; // laser on delay
  } else {
    //ndelay2=0;
    ndelay = delay;
  }
}

#define timer_set(a) timer_set2(a,0)
#endif


//#elif defined(__ARM__)//avr


#ifndef USETIMEROK
#undef USETIMER1
void timer_init() {};
void timer_set2(int32_t delay, int32_t delayl) {};
#endif
