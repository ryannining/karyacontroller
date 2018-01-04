#include<math.h>
#include<stdint.h>
#include "config_pins.h"
#define NUMAXIS 3

typedef struct {
    float  fx[NUMAXIS];
    float  fs,fn,fe,ac1,ac2;
    int32_t cx[NUMAXIS];
    int32_t sx[NUMAXIS];
    int32_t dx[NUMAXIS];
    int32_t totalstep,rampup,rampdown;       
    int8_t  bpos,planstatus,col,fastaxis,status;
} tmove;

extern tmove *m;
extern float x[NUMAXIS];
extern float homingspeed;
extern float homeoffset[NUMAXIS];
extern float jerk[NUMAXIS]; 
extern float accel[NUMAXIS];
extern float maxf[NUMAXIS];
extern float stepmmx[NUMAXIS];
extern tmove move[NUMBUFFER];
extern float cx1,cy1,cz1,lf;
extern int32_t head,tail;
extern uint8_t checkendstop;
extern uint8_t endstopstatus[NUMAXIS];

#define nextbuff(x) ((x) < NUMBUFFER-1 ? (x) + 1 : 0)
#define prevbuff(x) ((x) > 0 ? (x) - 1 : NUMBUFFER-1)
#define ACCELL(v0,v1,a) v0<v1?a:-a
#define degtorad(x) x*22/(7*180);


static int32_t ramplen(float v0,float v1,float a ,float stepmm)
{
     float t=(v1-v0)/a;
     return fabs((v0*t+0.5*a*t*t)*stepmm);
}
static float speedat(float v0,float a,float s,float stp=1) 
{
    return sqrt(a*2*s/stp+v0*v0);
}
static float accelat(float v0,float v1,float s)
{
    //v1=sqr(a*2*s+v0*v0)
    //a=(v1*v1-v0*v0)/(2*s)
    return (v1*v1-v0*v0)/(2*s);
}
 
extern int32_t motionrunning; 
extern void motionloop();
extern void waitbufferempty();
extern void needbuffer();
extern int32_t startmove();
extern void initmotion();
extern void addmove(float f,float x2,float y2 ,float z2 );
extern void homing(float x,float y ,float z );
extern float tick,fscale;
extern int32_t bufflen();
extern int32_t docheckendstop();

extern float homingspeed;
extern float homeoffset[NUMAXIS];


