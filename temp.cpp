#include "config_pins.h"
#include "common.h"
#include "temp.h"
#include "motion.h"


uint32_t next_temp;
uint16_t ctemp = 0;
double Setpoint, Input, Output;
int wait_for_temp = 0;

int fan_val = 0;
void setfan_val(int val) {
#ifdef fan_pin
    pinMode(fan_pin, OUTPUT);
  analogWrite(fan_pin, val);
//  digitalWrite(fan_pin, val);
  fan_val = val;
#endif
}


#if defined(temp_pin) && !defined(ISPC)
#include <PID_v1.h>



//Specify the links and initial tuning parameters

PID myPID(&Input, &Output, &Setpoint, 8, 2, 12, DIRECT); //2, 5, 1, DIRECT);



#if defined(__AVR__) && defined(ISRTEMP)
int vanalog[8];
int adcpin;

ISR (ADC_vect)
{
  uint8_t low, high;

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low = ADCL;
  high = ADCH;

  vanalog[adcpin] = (high << 8) | low;
  /*Serial.print(adcpin);
  Serial.print(":");
  Serial.print(vanalog[adcpin]);
  Serial.write('\n');
*/
}  // end of ADC_vect
#endif

void set_temp(float set) {
  Setpoint = set;
  pinMode(heater_pin, OUTPUT);
  analogWrite(heater_pin, 0);

}
void init_temp()
{
  //initialize the variables we're linked to

  //turn the PID on

  myPID.SetMode(AUTOMATIC);

  next_temp = micros();
  set_temp(0);

#if defined( __AVR__) && defined(ISRTEMP)
  
  ADCREAD(temp_pin)
#elif defined(ESP8266)
#endif


}

float read_temp(int32_t temp) {

  for (int j = 1; j < NUMTEMPS; j++) {
    if (pgm_read_word(&(temptable[j][0])) > temp) {
      // Thermistor table is already in 14.2 fixed point
      // Linear interpolating temperature value
      // y = ((x - x₀)y₁ + (x₁-x)y₀ ) / (x₁ - x₀)
      // y = temp
      // x = ADC reading
      // x₀= temptable[j-1][0]
      // x₁= temptable[j][0]
      // y₀= temptable[j-1][1]
      // y₁= temptable[j][1]
      // y =
      // Wikipedia's example linear interpolation formula.
      temp = (
               //     ((x - x₀)y₁
               ((uint32_t)temp - pgm_read_word(&(temptable[j - 1][0]))) * pgm_read_word(&(temptable[j][1]))
               //                 +
               +
               //                   (x₁-x)
               (pgm_read_word(&(temptable[j][0])) - (uint32_t)temp)
               //                         y₀ )
               * pgm_read_word(&(temptable[j - 1][1])))
             //                              /
             /
             //                                (x₁ - x₀)
             (pgm_read_word(&(temptable[j][0])) - pgm_read_word(&(temptable[j - 1][0])));
      return float(temp) / 4.0;
    }
  }
  return 0;
}


void temp_loop(uint32_t cm)
{
  if ((next_temp < cm) && (cm - next_temp < 1000000)) {
    next_temp = cm + 1000000; // each half second
  if (Setpoint==0)return;
int v=0;
#if defined( __AVR__) && defined(ISRTEMP)
    // automatic in ESR
    ADCREAD(temp_pin)
    v=vanalog[temp_pin];
#else
    v = analogRead(temp_pin) >> ANALOGSHIFT;
#endif

    ctemp = (ctemp + v * 7) / 8;
    Input =  read_temp(ctemp);
#ifdef fan_pin
    if ((Input > 80) && (fan_val < 50)) setfan_val(255);
#endif
    if (Setpoint > 0) {
#ifdef heater_pin
      //pinMode(heater_pin, OUTPUT);

      myPID.Compute();
      #ifdef ESP8266
      analogWrite(heater_pin, Output*4);
      #elif defined __ARM__
      analogWrite(heater_pin, Output*2);
      #else
      analogWrite(heater_pin, Output*17/20);
      #endif
      if (wait_for_temp ) 
      zprintf(PSTR("Temp:%f @%f\n"), ff(Input),ff(Output));

#endif
    }
  }
}
int temp_achieved() {
  return Input >= Setpoint- 10;

//  return fabs(Input - Setpoint) < 10;
}

#else

int temp_achieved() {
  return 1;
}
void set_temp(float set) {
}

void init_temp()
{
}
void temp_loop(uint32_t cm)
{
}
#endif

