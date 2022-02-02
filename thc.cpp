
#include "common.h"

#ifdef ANALOG_THC
#define NUMTHCBUF 511
#define nextthc(x) ((x+1)&NUMTHCBUF)
#define prevthc(x) ((x-1)&NUMTHCBUF)

int16_t thcbuff[NUMTHCBUF+1];

int thcstep = 0;
int thcstepmax = 0;
int thcread = 0;
uint32_t lasthcok;

int thctail=0;
int thchead=0;
int thcsep,thcspeed;
int thcdir;
void thc_init()
{
  lasthcok=0;
  thcstep = 0;
  thcread = 0;
  thcstepmax=labs(stepmmx[2])*2;
  thcsep=0;  
  thctail=0;
  thchead=0;
  thcdir=-10;
  thcspeed=3; // 0 = full speed
}

void push_thc(int v){
  int t=nextthc(thchead);
  if (t==thctail)thctail=nextthc(thctail);
  thchead=t;
  thcbuff[t]=v;
}
int laston=0;
void thc_loop(uint32_t m)
{
  extern int laserwason;
  if (laston==0 && laserwason){
    thchead=0;
    thctail=0;
    push_thc(0);
  }
  laston=laserwason;
  if (m - lasthcok < 60000)return;
  lasthcok = m;
  thcread = analogRead(A0);
    
  if (laserwason>0){
      extern int thc_ofs,thc_up,thc_ofs0;
      extern int odir[4];
      thcdir=0; // default no movement
      // offset negative disable thc
      // if change is greater than 15x offset, then disable thc too
      int thc_up1=thc_up+(thc_up<0?-1:1)*thc_ofs0;
      if ((thcread<800) && (thcread>100)&& (thc_ofs>0)){	        
        int cdir=1;
        if (thc_up1<0)cdir=-1; // if ref is negative, invert movement up/down
			  
        if (thcread > labs(thc_up1)+thc_ofs)thcdir = -cdir; // move up
			  else if (thcread < labs(thc_up1)-thc_ofs)thcdir = cdir; // move down
			  
        thcdir*=odir[2]; // set the direction according the step/mm settings (if negative invert movement)
		  }
		  // limit z movement 2mm each sensor reading
		  thcstep=labs(stepmmx[2])*2; 
      
    // save value to buffer for graph plot 
      push_thc(thcread);
  } else thcread=0;
  
}

String formatthc(){
	String res;
  extern int thc_ofs,thc_up,thc_ofs0;
	if (thchead==thctail)return String("[]");
	res="["+String(thc_up+(thc_up<0?-1:1)*thc_ofs0)+","+String(thc_ofs);
	int t=thctail;
	while (t!=thchead){
		t=nextthc(t);
		res+=","+String(thcbuff[t]);
	}
	res+="]";
	return res;
	
}

#else

int thc_init() {
}
void thc_loop(uint32_t m)
{
}
#endif
