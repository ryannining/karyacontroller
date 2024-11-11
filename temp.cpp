
#include "common.h"
#include "temp.h"

#include "pid.h"

int water_pin=-1;
int ltemp_pin=-1;
int temp_limit=55;
int BUZZER_ERR=-1;

#ifdef READTEMP
#include <OneWire.h> 
#include <DallasTemperature.h>
OneWire oneWire(0); 
DallasTemperature sensors(&oneWire);
#endif


float HEATINGSCALE = 1;
int water=1000;
int machinefail=10;
int machinehasfail=0;




uint32_t next_temp;
uint16_t ctemp = 0;
double Setpoint, Input, xInput, Output;
float tbang = 6;

int HEATING = 0;

int fan_val = 0;
void setfan_val(int val) {

}


#include "pid.h"



//Specify the links and initial tuning parameters

//PID myPID(&Input, &Output, &Setpoint, 8, 2, 12, DIRECT); //2, 5, 1, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 1, 2, 13, DIRECT); //2, 5, 1, DIRECT);



int WindowSize = 1500;
unsigned long windowStartTime;
void set_temp(float set) {
}
void BuzzError(bool v){
	if (BUZZER_ERR>-1){
		digitalWrite(BUZZER_ERR,v);
	}
}
int fail1=10;
void init_temp()
{
  //initialize the variables we're linked to
  //turn the PID on
  //myPID.SetMode(AUTOMATIC);
  next_temp = micros();
  set_temp(0);
// have temp sensor (and water and buzzer error)

#ifdef READTEMP
  if (ltemp_pin>-1){
	  oneWire.begin(ltemp_pin);
	  sensors.begin();
	  sensors.setWaitForConversion(false);
	  //sensors.setResolution(9);
	  sensors.requestTemperatures();
	}
#endif
  fail1=3;
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


void temp_loop(uint32_t cm)
{
  if (cm - next_temp > 450000) { // 900ms
    float sec = (cm - next_temp) / 1000000.0;
    if (sec > 1)sec = 1;
    next_temp = cm; // each 0.5 second
	int v=0;
	if (water_pin>-1) {
		int w=analogRead(water_pin);
		water= water*0.8+w*0.2;
	} else {
		water=1000; // assume water OK
	}
	#ifdef READTEMP
	if (ltemp_pin>-1){
		if (sensors.isConversionComplete())	{
			float t=sensors.getTempCByIndex(0);
			//t=27+(t-27)*0.9; // calibration 
			//if (t>20){
				fail1=3;
				//if (Input>20){
				//	if (fabs(t-Input)>5)t=Input; // skip sudden change
				//	if (fabs(t-Input)>1.5) t=Input+(t-Input)*0.1;
				//}	 		
				Input=Input*0.8+t*0.2; // averaging
				
				Input=t;
				#ifdef PLOTTING
				push_temp(Input);
				#endif
			//} else {
				fail1--;			
			//}
			if (fail1)sensors.requestTemperatures();
		} else {
			fail1--;
		}
		if (!fail1) {init_temp();}
	}
	#endif
	extern void dopause(int tm,bool stopping);
	if (water<600 || Input>temp_limit){
		if (--machinefail<8) {
			BuzzError(machinefail & 1) ;
			if (machinefail<0){
				extern int uncompress,ispause;
				if (uncompress && !ispause){
					dopause(0,false);// pause and wait until all sensors ok
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
