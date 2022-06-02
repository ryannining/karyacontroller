
#include "common.h"
#include "temp.h"

#include "pid.h"

int water_pin=-1;
int ltemp_pin=-1;
int temp_limit=55;
int BUZZER_ERR=-1;

#ifdef DS18B20
#include <OneWire.h> 
#include <DallasTemperature.h>
OneWire oneWire(D2); 
DallasTemperature sensors(&oneWire);
#endif

#ifdef EMULATETEMP
#undef ISRTEMP
float emutemp = 30;
#endif
float HEATINGSCALE = 1;
int water=1000;
int machinefail=10;
int machinehasfail=0;




uint32_t next_temp;
uint16_t ctemp = 0;
double Setpoint, Input, xInput, Output;
float tbang = 6;
int wait_for_temp = 0;
int HEATING = 0;

int fan_val = 0;
void setfan_val(int val) {

}


#if defined(heater_pin) || defined(DS18B20)
#include "pid.h"



//Specify the links and initial tuning parameters

PID myPID(&Input, &Output, &Setpoint, 8, 2, 12, DIRECT); //2, 5, 1, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 1, 2, 13, DIRECT); //2, 5, 1, DIRECT);



int WindowSize = 1500;
unsigned long windowStartTime;
void set_temp(float set) {
  if (set > MAXTEMP)set = MAXTEMP;
  Setpoint = set;
  windowStartTime = millis();

  HEATER(LOW);
}
void BuzzError(bool v){
	if (BUZZER_ERR>-1){
		digitalWrite(BUZZER_ERR,v);
	}
}
void init_temp()
{
  //initialize the variables we're linked to

  //turn the PID on

  myPID.SetMode(AUTOMATIC);

  next_temp = micros();
#ifdef heater_pin
  xpinMode(heater_pin, OUTPUT);
#endif
  set_temp(0);
// have temp sensor (and water and buzzer error)

#ifdef DS18B20
  if (ltemp_pin>-1){
	  oneWire.begin(ltemp_pin);
	  sensors.begin();
	  sensors.setWaitForConversion(false);
	  //sensors.setResolution(12);
	  sensors.requestTemperatures();
	}
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
#ifdef PLOTTING
#define NUMTEMPBUF 127
#define nexttemp(x) ((x+1)&NUMTEMPBUF)
#define prevtemp(x) ((x-1)&NUMTEMPBUF)
int temptail=0;
int temphead=0;
int16_t tempbuff[NUMTEMPBUF+1];
long lastpush=0;
bool lastunc=false;
void push_temp(float v){
  extern int uncompress;
  if (lastunc!=(uncompress==1)){
	if (!lastunc) {
		temptail=0;
		temphead=0;
	}  
	lastunc=uncompress==1;
  }
  if (millis()-lastpush>8000){
	  lastpush=millis();
	  int t=nexttemp(temphead);
	  if (t==temptail)temptail=nexttemp(temptail);
	  temphead=t;
	  tempbuff[t]=v*100;
  }
}

String formattemp(){
	String res;
	if (temphead==temptail)return String("[]");
	res="[";
	int t=temptail;
	while (t!=temphead){
		t=nexttemp(t);
		res+=","+String(tempbuff[t]);
	}
	res+="]";
	return res;
	
}
#endif
uint32_t ectstep2 = 0;
int tmc1 = 0;
/*
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
    float tt = 0;
    if (HEATING) {
      if (emutemp < 200)dt = pow(200.0 - emutemp, 0.2);
      else dt = 1; //pow((250.0-emutemp)/50.0,2);
      emutemp += dt * sec;
    } else tt = tbang;
    // heat dissipation to air, can be tweak using eeprom ET
    dt = -pow(emutemp, 2.97) * 0.0000001;
    if (ectstep2 != ectstep) {
      // if there is printing / extruding then need to adjust, since the flow of filament take the heat away
      uint32_t et = (ectstep2 - ectstep);
      if (et < 0)et = 0;
#define stmax 16
      float tt2;
      if (et > stepmmx[3] * stmax)tt2 = stmax; else tt2 = et / stepmmx[3];
      tt += tt2 * HEATINGSCALE;
      //zprintf(PSTR("%d\n"),fi(et));
      ectstep2 = ectstep;
    }
    dt *= tt;
    emutemp += dt * sec;
    // clamp the temperature calculation
    if (emutemp > 250)emutemp = 250;
    if (emutemp < 30)emutemp = 30;
    Input = emutemp;
#ifdef temp_pin
    tmc1++;
    if (tmc1 > 40) {
      tmc1 = 0;
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


    v = analogRead(temp_pin) >> ANALOGSHIFT;


#ifdef ESP8266

    //v = v * 3.3 + 120; //200K resistor
    v = v * 1 + 120; //22K resistor

#endif

    //    ctemp = v;//(ctemp * 2 + v * 6) / 8; // averaging
    ctemp = (ctemp + v) / 2; // averaging
    Input =  read_temp(ctemp);
    //Input = 100;
#endif

#ifdef fan_pin
    if ((Input > 80) && (fan_val < 50)) setfan_val(255);
#endif
    //if (Setpoint >= 0)
    //{
#ifdef heater_pin
      //xpinMode(heater_pin, OUTPUT);
      myPID.Compute();
#define BANGBANG

#ifdef BANGBANG
      myPID.SetOutputLimits(0, WindowSize);
      goto bang;
#warning USING BANG HEATER
#endif

#ifndef BANGBANG

#if defined __ARM__
      analogWrite(heater_pin, Output * 2);
#else
      analogWrite(heater_pin, Output * 17 / 20);
#endif
      if (wait_for_temp ) {
        //zprintf(PSTR("Temp:%f @%f\n"), ff(Input), ff(Output));
      }
#endif
    //}
  }
  return;
#endif

#if defined(BANGBANG) && defined(heater_pin)
bang:
  /** **********************************************
       turn the output pin on/off based on pid output
     ************************************************ /
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }

  HEATING = Output > now - windowStartTime;
  HEATER(HEATING);
#endif

}

*/
int fail1=10;
void temp_loop(uint32_t cm)
{
  if (cm - next_temp > TEMPTICK) {
    float sec = (cm - next_temp) / 1000000.0;
    if (sec > 1)sec = 1;
    next_temp = cm; // each 0.5 second
	int v=0;
	if (water_pin>-1) {
		int w=analogRead(water_pin);
		if (w>800)water= water*0.95+w*0.05; else water= water*0.5+w*0.5;
	} else {
		water=40; // assume water OK
	}
	
	#ifdef DS18B20
	if (ltemp_pin>0){
		if (sensors.isConversionComplete())	{
			float t=sensors.getTempCByIndex(0);
			//t=27+(t-27)*0.9; // calibration 
			if (t>20){
				//if (Input>20){
				//	if (fabs(t-Input)>5)t=Input; // skip sudden change
				//	if (fabs(t-Input)>1.5) t=Input+(t-Input)*0.1;
				//}	 		
				if (t>10)Input=Input*0.8+t*0.2; // averaging
				
				Input=t;
				#ifdef PLOTTING
				push_temp(Input);
				#endif
			} else {
				//sensors.begin();
				if (t<0) t=30;			
				Input=t;
			}
			sensors.requestTemperatures();
			fail1=10;
		} else {
			fail1--;
			if (fail1==0) {fail1=10;sensors.begin();sensors.requestTemperatures();}
			Input=0;
		}
	}
	#else  
	v = analogRead(temp_pin);// >> ANALOGSHIFT;
    //v = v * 3.3 + 120; //200K resistor
    //v = v * 1 + 120; //22K resistor
    //v = v * 0.3 + 120; //22K resistor


    //    ctemp = v;//(ctemp * 2 + v * 6) / 8; // averaging
    ctemp = (ctemp + v) / 2; // averaging
    Input =  ctemp;//read_temp(ctemp);
    //Input = 100;
    #endif
	  extern void dopause(int tm);
	  if (water>1000 || Input>temp_limit){
		if (--machinefail<8) {
			BuzzError(machinefail & 1) ;
			if (machinefail<0){
				extern int uncompress,ispause;
				if (uncompress && !ispause){
					dopause(0);// pause and wait until all sensors ok
					machinehasfail=10;
				}
				machinefail=12;
			}
		}
		
	  } else {
		if (machinehasfail==0) {  
			BuzzError(LOW);
			if (++machinefail>20)machinefail=20;
		} else {
			BuzzError(machinehasfail & 1);
			if (--machinehasfail<2){
				machinehasfail=0;
				//dopause(0); // resume job
			}
		}
	  }
		
   }
}

int temp_achieved() {
  return Input >= Setpoint;

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
void BuzzError(bool v){
}

String formattemp(){return "";}
#endif
