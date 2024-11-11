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
  extern int lasermode,uncompress;  
  if (uncompress && v==0 && lasermode==0)v=255; // make sure its running when running job on router
  if (v > 255)v = 255;
  if (v < 0)v = 0;
  laserOn = v > 0;
  // 
  

  // turn on tool pwm pin != tool pin 
  if (pwm_pin!= atool_pin && !uncompress) {
    TOOL1((laserOn && lasermode!=1?TOOLON:!TOOLON));
  }
  constantlaserVal = v;
  lastS = v;
}
void set_toolx(int v) { // 0-255

}
void pause_pwm(bool v) {
  in_pwm_loop = v;
}

void THEISR pwm_loop() {
  
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

void somedelay(int32_t n)
{
  auto m=micros();
  while ((micros()-m)<n){};

}

//#define somedelay(n) delayMicroseconds(n);
int dogfeed = 0;

#include<Arduino.h>
#define dogfeedevery 10200 // loop
// ESP8266 specific code here


void feedthedog()
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
  }
  
}


// ======================== TIMER for motionloop =============================


int busy1 = 0;

volatile uint32_t ndelay = 0;
uint32_t next_step_time;

uint32_t IRAM_ATTR timercode();



int tm_mode=0;
extern int lasermode;
extern int32_t pwm_val;
extern bool toolWasOn;
bool TMTOOL=false;
bool TMTOOL0=false;
uint32_t tool_duration = 0;  // Duration for tool ON in microseconds
bool tool_phase = false;     // false = motor step phase, true = tool off phase


uint8_t last_pwm_val = 255; 
uint32_t ttt;
IRAM_ATTR uint32_t timercode() {

  
  if (!tool_phase) {
    // Motor step phase
    ndelay = MINDELAY;

    coreloopm(); // Handle motor stepping
    // Calculate next timer interval
    if (pwm_val>0 && pwm_val<255) {
      if (pwm_val != last_pwm_val) {
          last_pwm_val = pwm_val;
          extern uint32_t stepdiv3a;
          // Calculate tool ON duration based on current step timing
          tool_duration = (stepdiv3a * pow(pwm_val*0.0039,Lscale));   
          //Serial.println(tool_duration);     
      } 
      // Partial power - set timer for tool ON duration
      TOOLPWM(TOOLON);
      tool_phase = true;
      uint32_t d=tool_duration;
      if (d>ndelay)d=ndelay;
      ndelay=ndelay-d;
      return d;

    }
    // 0% or 100% power - just handle motor steps
    TOOLPWM(pwm_val ==0 ? !TOOLON : TOOLON);
    return (ndelay);
  
  }
  // Tool off phase
  tool_phase = false;
  // Set timer for remaining step time
  TOOLPWM(!TOOLON);
  return (ndelay);
  
}

#ifdef ESP8266
#define USETIMEROK
IRAM_ATTR void tm() {
  noInterrupts();
  uint32_t d=timercode();
  timer1_write(d>MINDELAY?d:MINDELAY);
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


#ifdef RPM_COUNTER
  setup_RPM();
#endif
  set_tool(0);
  interrupts();
}

#endif // esp8266l
#ifdef ESP32
#define USETIMEROK
// alternative using polling system, slow, non accurate
uint32_t lT=0;
bool usepolling=false;
void IRAM_ATTR tmloop(uint32_t T) {
  if (T>lT){
    uint32_t d=timercode();
    if (d<MINDELAY)d=MINDELAY;
    lT=T+d;
  }
    // Update the timer for the next interval

}

// main timer function
#include "esp_timer.h"
#include "driver/gpio.h"

esp_timer_handle_t pulse_timer;
volatile bool pulseState = false;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR tm(void* arg) {
  uint32_t d=timercode();
  if (d<MINDELAY)d=MINDELAY;
  esp_timer_start_once(pulse_timer, d);
}


void timer_init()
{
  lT=micros()+1000;

  esp_timer_create_args_t timer_args;
  timer_args.callback = &tm;
  timer_args.arg = NULL;
  timer_args.dispatch_method = ESP_TIMER_TASK;    // Run in a task context
  timer_args.name = "core";

  // Create the high-resolution timer
  esp_err_t result = esp_timer_create(&timer_args, &pulse_timer);
  if (result != ESP_OK) {
      Serial.println("Failed to create timer");
      usepolling=true;
      return;
  }

    // Start the timer with the specified interval in microseconds
  esp_timer_start_once(pulse_timer, 10000);

}



#endif //esp32


// Laser constant burn timer
void IRAM_ATTR timer_set(int32_t delay)
{
  ndelay = delay; // delay total
}
