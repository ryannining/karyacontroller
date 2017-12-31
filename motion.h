#include<math.h>
#include<stdint.h>
#define numaxis 3
#define numbuffer 6

typedef struct {
  int32_t totalstep;
    double fx[numaxis];
    int32_t   cx[numaxis];
    int32_t     sx[numaxis];
    int32_t     dx[numaxis];
    double  fs,fn,fe,ac1,ac2;
    int32_t rampup,rampdown,status;       
    int32_t planstatus,col,fastaxis;
} tmove;

static int32_t head,tail=0;

#define nextbuff(x) ((x) < numbuffer ? (x) + 1 : 1)
#define prevbuff(x) ((x) > 1 ? (x) - 1 : numbuffer)
#define ACCELL(v0,v1,a) v0<v1?a:-a
#define degtorad(x) x*22/(7*180);


inline int32_t ramplen(double v0,double v1,double a ,double stepmm)
{
     double t=(v1-v0)/a;
     return fabs((v0*t+0.5*a*t*t)*stepmm);
}
inline double speedat(double v0,double a,double s) 
{
    return sqrt(a*2*s+v0*v0);
}
inline double accelat(double v0,double v1,double s)
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


