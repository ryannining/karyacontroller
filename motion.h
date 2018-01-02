#include<math.h>
#include<stdint.h>
#define numaxis 3
#define numbuffer 5

typedef struct {
    double  fx[numaxis];
    double  fs,fn,fe,ac1,ac2;
    int32_t cx[numaxis];
    int32_t sx[numaxis];
    int32_t dx[numaxis];
    int32_t totalstep,rampup,rampdown;       
    int8_t  bpos,planstatus,col,fastaxis,status;
} tmove;

extern tmove *m;
extern double x[numaxis];
extern double homingspeed;
extern double homeoffset[numaxis];
extern double jerk[numaxis]; 
extern double accel[numaxis];
extern double maxf[numaxis];
extern double stepmmx[numaxis];
extern tmove move[numbuffer];
extern double cx1,cy1,cz1,lf;
extern int32_t head,tail;
extern uint8_t checkendstop;
extern uint8_t endstopstatus[numaxis];

#define nextbuff(x) ((x) < numbuffer-1 ? (x) + 1 : 0)
#define prevbuff(x) ((x) > 0 ? (x) - 1 : numbuffer-1)
#define ACCELL(v0,v1,a) v0<v1?a:-a
#define degtorad(x) x*22/(7*180);


static int32_t ramplen(double v0,double v1,double a ,double stepmm)
{
     double t=(v1-v0)/a;
     return fabs((v0*t+0.5*a*t*t)*stepmm);
}
static double speedat(double v0,double a,double s,double stp=1) 
{
    return sqrt(a*2*s/stp+v0*v0);
}
static double accelat(double v0,double v1,double s)
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
extern void addmove(double f,double x2,double y2 ,double z2 );
extern void homing(double x,double y ,double z );
extern double tick,fscale;
extern int32_t bufflen();
extern int32_t docheckendstop();

extern double homingspeed;
extern double homeoffset[numaxis];


