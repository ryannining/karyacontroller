#include <stdlib.h>
#include <stdio.h>

#include <stdint.h>
#include "common.h"
#include "timer.h"

long spindle_pwm = 0;
int min_pwm_clock=0; // 2us to turn on anything
int zerocrosspin;
bool TOOLON=HIGH;
bool TOOLONS[3];
//#define motorz_servo
#ifdef motorz_servo
  #define motorz_servo_pin zstep
  long motorz_pwm = 0;
  // range 50mm, mean pwm pulse 1ms = -25 1.5=0 2ms = 2.5
  float motorz_zero = 0; // actual position must be added with this, when job start, must be set
  #define motorz_range 50.0
  #define motorz_range_min -(motorz_range/2)
  #define motorz_range_max (motorz_range/2)
  
   
  uint32_t nextz_l;
  bool motorz_servo_state=LOW;
#endif


extern int lastS;
bool RPM_PID_MODE = false;


#ifdef RPM_COUNTER

	#include "pid.h"
	double rSetpoint, rInput, rOutput;

	//Specify the links and initial tuning parameters

	PID RPM_PID(&rInput, &rOutput, &rSetpoint, 8, 2, 12, DIRECT); //2, 5, 1, DIRECT);

	uint32_t rev = 0;
	uint32_t lastMillis = 0;
	uint32_t RPM = 0;
	int avgpw = 0;
	int avgcnt = 0;
	void THEISR isr_RPM() {
	  rev++;
	}

	uint32_t get_RPM() {
	  uint32_t milli = millis();
	  uint32_t delta = milli - lastMillis;
	  if (delta > 100) {
		lastMillis = milli;
		RPM = (rev * 60 * 1000 / delta);
		rev = 0;
		if (RPM_PID_MODE) {
		  rSetpoint = lastS * 100;
		  rInput = RPM;
		  RPM_PID.Compute();
		  if (lastS == 0)rOutput = 0;
		  avgpw = (avgpw + rOutput * 0.01);
		  avgcnt++;
	#ifdef AC_SPINDLE
		  spindle_pwm = 10000 - pow(rOutput * 0.0001, 2) * 10000;
	#else
		  spindle_pwm = rOutput;
	#endif
		}
	  }
	  return RPM;
	}
	void setup_RPM() {
	  attachInterrupt(digitalPinToInterrupt (RPM_COUNTER), isr_RPM, RISING);
	  lastMillis = millis();
	  RPM_PID.SetMode(AUTOMATIC);
	  RPM_PID.SetOutputLimits(0, 10000);
	}

#endif



uint32_t next_l = 0;
bool spindle_state = LOW;
int spindle_pct;

//PWM Freq is 1000000/20000 = 50Hz , 20mms pulse
// for servo, the LSCALE at least 10%

#define FREQPWM 400

int X9pos=100;


#define PERIODPWM (1000000/FREQPWM)
#define BYTETOPERIOD PERIODPWM/255
#define PERIODTOPCA (180/PERIODPWM)



bool in_pwm_loop = false;
void set_tool(int v){
  if (v > 255)v = 255;
  if (v < 0)v = 0;
  laserOn = v > 0;
  // 
  extern int lasermode,uncompress;
  // turn on tool pwm pin != tool pin 
  if (pwm_pin!= atool_pin && !uncompress) {
    TOOL1((laserOn && lasermode!=1?TOOLON:!TOOLON));
  }
  constantlaserVal = v;
  lastS = v;
}
void set_toolx(int v) { // 0-255
/*  
  if (v > 255)v = 255;
  if (v < 0)v = 0;
#ifdef PLASMA_MODE
	// no need pwm for plasma
	if (v > 10)v = 255;
	xdigitalWrite(spindle_pin,v>10);
	lastS = v;
#else  
	  next_l = micros();
	  {
		spindle_pct = v * 0.3922; // 0 - 100
		//if (! RPM_PID_MODE)
		lastS = v;
		extern float Lscale;
		float vf;
		if (int(100 * Lscale) == 0) {
		  vf = v > 5 ? 255 : 0;
		} else {
		  vf = fabs(v * Lscale);
		}
	      #ifdef pinX9C
	      extern void IR_end();
	      extern void IR_setup();
		    int newX9pos=spindle_pct;
		    if (newX9pos!=X9pos){
		      IR_end();

		    // set direction
		      int np=newX9pos;
		      newX9pos=(newX9pos==0?-100:X9pos);
		      
		      X9pos=np;
		      pinMode(pinX9C,OUTPUT);
		      xdigitalWrite(pinX9C,newX9pos>0?1:0);
		      #ifdef spindle_pin
		      #define pot_pin spindle_pin
		      #elif defined(laser_pin)
		      #define pot_pin laser_pin
		      #endif
		      somedelay(1);
		      pinMode(pot_pin,OUTPUT);
		      
		      
		      for (int i=abs(newX9pos);i>0;i--){
			// step the potensio
			xdigitalWrite(pot_pin,1);
			somedelay(1);
			xdigitalWrite(pot_pin,0);			
			somedelay(1);
		      }
		      IR_setup();
		    }
	      #else
		spindle_pwm = fmin(PERIODPWM, vf * BYTETOPERIOD);
		if (Lscale >= 0)spindle_pwm = PERIODPWM - spindle_pwm; // flip if inverse
		spindle_state = LOW;
		#ifdef spindle_pin
			xdigitalWrite(spindle_pin, v>10);
		#endif
	      #endif
	  
	#ifdef PCA9685
		pwm.setPWM(spindle_servo_pin, 0, pulseWidth(spindle_pwm * PERIODTOPCA)); // set 0 - 4095
		return;
	#endif
	  }
	  if (in_pwm_loop)return;
	#ifdef spindle_pin
	  pinMode(spindle_pin, OUTPUT);
	  //xdigitalWrite(spindle_pin, HIGH);
	#endif
#endif
*/
}
void pause_pwm(bool v) {
  in_pwm_loop = v;
}

void THEISR pwm_loop() {
  /*
#ifndef PLASMA_MODE
	#ifdef RPM_COUNTER
	  get_RPM();
	#endif
	if (in_pwm_loop)return;
	in_pwm_loop = true;



	uint32_t pwmc = micros(); // next_l contain last time target for 50Hz
	#if defined(spindle_pin) && !defined(pinX9C)
	    if (!spindle_state && (pwmc - next_l > spindle_pwm)  && (spindle_pwm < PERIODPWM)) { // if  current time - next time > delta time pwm, then turn it on
		  spindle_state = HIGH;
		  xdigitalWrite(spindle_pin, HIGH);
	    }

	    // if use zero_cross then in_pwm_loop will be true until a trigger happened
	    // basically replace all below using interrupt trigger
	  #ifndef ZERO_CROSS_PIN
	    if (pwmc - next_l >= PERIODPWM) { // 50hz on wemos then turn off
		  next_l = pwmc;
		  spindle_state = LOW;
		  xdigitalWrite(spindle_pin, LOW);
	    }
	  #endif
	#endif
	/*
	#ifdef motorz_servo
	    if (!motorz_servo_state && (pwmc - nextz_l > motorz_pwm)) { // if  current time - next time > delta time pwm, then turn it on
		  motorz_servo_state = HIGH;
		  xdigitalWrite(motorz_servo_pin, !HIGH);
	    }
	    if (pwmc - nextz_l > 19999) { // 50hz on wemos then turn off
		  nextz_l = pwmc;
		  motorz_servo_state = LOW;
		  xdigitalWrite(motorz_servo_pin, !LOW);
		  // update motor z pwm / position
		  float cz = info_z_s * perstepz+motorz_zero;
		  if (cz<motorz_range_min)cz=motorz_range_min;
		  if (cz>motorz_range_max)cz=motorz_range_max;
		  // map to pwm 1ms to 2ms
		  motorz_pwm =  100+(cz-motorz_range_min)*2500/motorz_range;
	    }
	
	#endif
	*-/  
	in_pwm_loop = false;
#endif
*/
}


void servo_set(int angle) {
#ifdef servo_pin
  pwm.setPWM(servo_pin, 0, spindle_pwm * 0.205); // set 0 - 4095
  return;
#endif
}

void THEISR servo_loop() {
  //pwm_loop();
}

int somedelay(int32_t n)
{
/*  
  float f = 0;
  int m = 100;

  while (m--) {
    int nn = n;
    while (nn--) {
      f += n;
      asm("");
    }
  }
  return f + n;
*/
  auto m=micros();
  while ((micros()-m)<n){};
}

//#define somedelay(n) delayMicroseconds(n);
int dogfeed = 0;

#include<Arduino.h>
#define dogfeedevery 2200 // loop
// ESP8266 specific code here


int feedthedog()
{
  if (dogfeed++ > dogfeedevery) {
    dogfeed = 0;
#if defined(ESP8266)
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

int busy1 = 0;
volatile uint32_t ndelay = 0;
volatile uint32_t n1delay = 0;
uint32_t next_step_time;

inline int THEISR timercode();
// -------------------------------------   ESP8266  ----------------------------------------------------------
#ifdef ESP8266
#define USETIMEROK
#define MINDELAY 100
#define usetmr1

int tm_mode=0;
extern int lasermode;
extern uint32_t pwm_val;
extern bool toolWasOn;
bool TMTOOL=false;
bool TMTOOL0=false;
void THEISR tm01()
{
  noInterrupts();
  TMTOOL0=!TMTOOL0;
  uint32_t t0=ESP.getCycleCount();
  if (pwm_val>0){
    if (pwm_val>200){
      TOOLPWM(TOOLON);
      t0+=100000;
      // always on
    } else {
      TOOLPWM(TMTOOL0);
      //int v=(pwm_val*pwm_val)>>2;
      int v=(pwm_val)<<10;
      t0+=(min_pwm_clock+1)*40+(TMTOOL0?v:300000-v);
    }
  } else {
    TOOLPWM(!TOOLON);
    t0+=100000;
  }
  timer0_write(t0);
  interrupts();
}
void THEISR tm()
{
  noInterrupts();
  /*
  int d=0;
  // laser timer
  if (tm_mode==1){
    tm_mode=0;
    // Turn off laser
    d=n1delay;
  }
  if (d==0) {
  // motor timer
    d = timercode();
    tm_mode=n1delay>0?1:0;
  } else {     
    //TMTOOL=!TOOLON;
    //TOOL1(!TOOLON);
  }*/
  int d = timercode();
  
  timer1_write(d);
  //TOOL1(TMTOOL)
  interrupts();
}

void timer_init()
{
  //Initialize Ticker every 0.5s
  noInterrupts();

  timer1_isr_init();
  timer1_attachInterrupt(tm);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(1000); //200 us

  timer0_isr_init();
  timer0_attachInterrupt(tm01);
  timer0_write(ESP.getCycleCount() + 80 * 10); //10 us


#ifdef RPM_COUNTER
  setup_RPM();
#endif
  set_tool(0);
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


void THEISR xtimerAlarmWrite(xhw_timer_s *timer, uint64_t alarm_value) {
  timer->dev->alarm_high = (uint32_t) (alarm_value >> 32);
  timer->dev->alarm_low = (uint32_t) alarm_value;
  timer->dev->config.autoreload = true;
  timer->dev->config.alarm_en = 1;
}

void THEISR tm() {
  portENTER_CRITICAL_ISR(&timerMux);
  int d = timercode();

  xtimerAlarmWrite((xhw_timer_s*)timer, d < 6 ? 6 : d);

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

// ======= the same code for all cpu ======

/*
inline int THEISR timercode() {
  if (ndelay < 30000) {
    ndelay = MINDELAY;
    coreloopm();
  } else {
    ndelay -= 30000;
  }
  //pwm_loop();
  return ndelay >= 30000 ? 30000 : ndelay;
}
*/
inline int THEISR timercode() {
  ndelay = MINDELAY;
  n1delay= 0;

  coreloopm();
  return ndelay;
}

// ==============================================

// 

// Laser constant burn timer
void THEISR timer_set(int32_t delay)
{
  n1delay=0; // delay laser
  ndelay = delay; // delay total
}

// Laser constant burn timer
void THEISR timer_set2(int32_t delay1,int32_t delay2)
{
  // delay1 delay of the laser
  // delay2 delay total
  if (delay1>0){
    delay1+=min_pwm_clock;
    if (delay1>delay2)delay1=delay2;
    delay2-=delay1;
    n1delay= delay2; // laser on delay
    ndelay = delay1; // rest delay (can be 0)
    //TMTOOL=TOOLON;
    TOOL1(TOOLON);
  } else {
    //TMTOOL=!TOOLON;
    n1delay=0;
    ndelay = delay2; // rest relay (can be 0)
  }  
}

