#include "config_pins.h"
#include "common.h"
#include "temp.h"
#include "timer.h"
#include "motion.h"
#include "pid.h"


#ifdef EMULATETEMP
#undef ISRTEMP
float emutemp = 30;
#endif




uint32_t next_temp;
uint16_t ctemp = 0;
double Setpoint, Input, xInput, Output;
float tbang = 6;
int wait_for_temp = 0;
int HEATING = 0;

int fan_val = 0;
void setfan_val(int val) {
#ifdef fan_pin
  pinMode(fan_pin, OUTPUT);
#ifdef usetmr1
  digitalWrite(fan_pin, val);
#else
  analogWrite(fan_pin, val);
#endif
  //  digitalWrite(fan_pin, val);
  fan_val = val;
#endif
}


#if defined(heater_pin) && !defined(ISPC)
#include "pid.h"



//Specify the links and initial tuning parameters

PID myPID(&Input, &Output, &Setpoint, 8, 2, 12, DIRECT); //2, 5, 1, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 1, 2, 13, DIRECT); //2, 5, 1, DIRECT);



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

int WindowSize = 1500;
unsigned long windowStartTime;
void set_temp(float set) {
  if (set > MAXTEMP)set = MAXTEMP;
  Setpoint = set;
  windowStartTime = millis();
  xpinMode(heater_pin, OUTPUT);

  xdigitalWrite(heater_pin, 0);
}
void init_temp()
{
  //initialize the variables we're linked to

  //turn the PID on

  myPID.SetMode(AUTOMATIC);

  next_temp = micros();
  set_temp(0);
#ifdef temp_pin

#if defined( __AVR__) && defined(ISRTEMP)

  ADCREAD(temp_pin)
#else
  pinMode(temp_pin, INPUT);
#endif
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

uint32_t ectstep2 = 0;
int tmc1 = 0;
void temp_loop(uint32_t cm)
{
  if (cm - next_temp > TEMPTICK) {
    float sec = (cm - next_temp) / 1000000.0;
    if (sec > 1)sec = 1;
    next_temp = cm; // each 0.5 second
    int v = 0;
#ifdef EMULATETEMP
    // emulated sensor using math formula, sec= delta time
    float dt;
    if (HEATING) {
      dt = pow(200 - emutemp, 0.2);
    } else {
      // heat dissipation to air, can be tweak using eeprom ET
      dt = -pow(emutemp, 3) * 0.0000001;
      float tt = tbang;
      if (ectstep2 != ectstep) {
        // if there is printing / extruding then need to adjust, since the flow of filament take the heat away
        uint32_t et = (ectstep2 - ectstep);
        if (et < 0)et = 0;
        if (et >stepmmx[3])et = stepmmx[3];
        //zprintf(PSTR("%d\n"),fi(et));
        tt += et * 68.0/stepmmx[3];
        ectstep2 = ectstep;
      }
      dt *= tt;
    }
    emutemp += dt * sec;
    // clamp the temperature calculation
    if (emutemp > 200)emutemp = 200;
    if (emutemp < 30)emutemp = 30;
    Input = emutemp;
#ifdef xtemp_pin
    tmc1++;
    if (tmc1 > 40) {
      tmc1=0;
      // for debugging, still read the sensor if available and reported as Tx:$$
      v = analogRead(temp_pin) >> ANALOGSHIFT;
      v = v + 120; //22K resistor
      ctemp = (ctemp + v) / 2; // averaging
      xInput =  read_temp(ctemp);
      // averaging with the emulated one if the real temperature is higher
      if (xInput > Input)Input = Input * 0.5 + xInput * 0.5;
    }
#endif

#else
    // real hardware sensor

#if defined( __AVR__) && defined(ISRTEMP)
    // automatic in ESR
    ADCREAD(temp_pin)
    v = vanalog[temp_pin];
#else
    v = analogRead(temp_pin) >> ANALOGSHIFT;
    //zprintf(PSTR("%d\n"),fi(v));
#endif
#ifdef ESP8266

    //v = v * 3.3 + 120; //200K resistor
    v = v * 1 + 120; //22K resistor

#endif

    //    ctemp = v;//(ctemp * 2 + v * 6) / 8; // averaging
    ctemp = (ctemp + v) / 2; // averaging
    Input =  read_temp(ctemp);
#endif

#ifdef fan_pin
    if ((Input > 80) && (fan_val < 50)) setfan_val(255);
#endif
    if (Setpoint > 0) {
#ifdef heater_pin
      //xpinMode(heater_pin, OUTPUT);

      myPID.Compute();
#define BANGBANG
#ifdef BANGBANG
      myPID.SetOutputLimits(0, WindowSize);
      goto bang;
#warning USING BANG HEATER
#endif


#if defined __ARM__
      analogWrite(heater_pin, Output * 2);
#else
      analogWrite(heater_pin, Output * 17 / 20);
#endif
      if (wait_for_temp ) {
        //zprintf(PSTR("Temp:%f @%f\n"), ff(Input), ff(Output));
      }
#endif
    }
  }
  return;
#ifdef BANGBANG
bang:
  /************************************************
       turn the output pin on/off based on pid output
     ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  HEATING =(Output > now - windowStartTime;
  xdigitalWrite(heater_pin, HEATING);
#endif

}
int temp_achieved() {
  return Input >= Setpoint - 10;

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

