#include "motion.h"
#include "gcode.h"
#include "common.h"
#include "config_pins.h"
#include "timer.h"
#include "temp.h"
#include <stdint.h>


#ifndef ISPC
#include<Arduino.h>
#include "eprom.h"
#include "gcodesave.h"
#include "ir_remote.h"

#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#endif // ispac


// bikin pusing !, buat check TIMER di AVR, malah freeze
//#define DEBUGLOOP

#ifdef DEBUGLOOP
#define LOOP_IN(n) zprintf(PSTR("LOOP %d IN "),fi(n));
#define LOOP_OUT(n) zprintf(PSTR("LOOP %d OUT "),fi(n));
#else
#define LOOP_IN(n)
#define LOOP_OUT(n)
#endif // debugloop


#ifdef USETIMER1
#ifdef __AVR__
#define CLOCKCONSTANT 2000000.f        // tick/seconds
#define DSCALE 0   // use 2Mhz timer need to shift right 2bit
#define DIRDELAY 4
#endif // avr

#ifdef __ARM__
#define CLOCKCONSTANT 8000000.f        // tick/seconds
#define DSCALE 0   // use 8Mhz timer shift 0bit
#define DIRDELAY 4 // usec
#endif // arm

#ifdef ESP8266
#define CLOCKCONSTANT 5000000.f        // tick/seconds
#define DSCALE 0   // use 5Mhz timer shift 0bit
#define DIRDELAY 4 // usec
#endif // esp

#ifdef  ESP32
#define CLOCKCONSTANT 80000000.f        // tick/seconds
#define DSCALE 0   // use 5Mhz timer shift 0bit
#define DIRDELAY 4 // usec
#endif // esp

#else // usetimer1
#define CLOCKCONSTANT 1000000.f        // tick/seconds

#define DSCALE 0  // 1mhz use micros shift 3bit
#define DIRDELAY 10
#endif // usetimer

#ifdef INVERTENDSTOP
#define ENDCHECK !
#else
#define ENDCHECK
#endif

#define ENDPIN INPUT_PULLUP


int MESHLEVELING = 0;
int vSUBPIXELMAX = 1;
int constantlaserVal = 0;
int laserOn = 0, isG0 = 1;
int babystep[4] = {0, 0, 0, 0};
uint8_t homingspeed;
int xback[4];
uint8_t head, tail, tailok;
int maxf[4];
int maxa[4];
float Lscale, f_multiplier,  e_multiplier;
int accel, zaccel;
int32_t xycorner, zcorner,  xyjerk, zjerk;
int i;
float ax_home[NUMAXIS];
float stepdiv, stepdiv2, stepdivL;
float wstepdiv2;
int32_t totalstep;
uint32_t bsdx[NUMAXIS];
int8_t  sx[NUMAXIS];
int32_t dlp, dln;
int8_t checkendstop, xctstep, yctstep, zctstep,  xcheckevery, ycheckevery, zcheckevery, echeckevery;
uint32_t ectstep = 0;
int16_t endstopstatus;
int8_t ishoming;
float axisofs[4] = {0, 0, 0, 0};
float F_SCALE = 1;
int8_t RUNNING = 1;
int8_t PAUSE = 0;
float stepmmx[4];
int odir[4] = {1, 1, 1, 1};
float retract_in, retract_out;
float retract_in_f, retract_out_f;
float cx1, cy1, cz1, ocz1, ce01, extadv;
tmove moves[NUMBUFFER];

#define sqr2(n) (n)*(n)


#include "nonlinear.h"

// to calculate delay from v^2, fast invsqrt hack
#ifdef __AVR__
//#define FASTINVSQRT  // less precice speed faster 4us

float InvSqrt(float x)
{

#ifdef FASTINVSQRT
  int32_t* i = (int32_t*)&x;           // store floating-point bits in integer
  *i = 0x5f335a86  - (*i >> 1);    // initial guess for Newton's method
#else // if FAST then bypass newtons method
  float xhalf = 0.5f * x;
  int32_t i = *(int32_t*)&x;            // store floating-point bits in integer
  i = 0x5f375a86  - (i >> 1);    // initial guess for Newton's method
  x = *(float*)&i;              // convert new bits into float
  x = x * (1.5f - xhalf * x * x); // One round of Newton's method , disable if want faster but not precise
#endif
  return x;
}
#else // avr
// other CPU use normal SQRT
#define InvSqrt(f) 1.0/f
#endif // avr

//#define xInvSqrt(n) n>1?stepdiv2*InvSqrt(n):stepdiv

//#define xInvSqrt(n) stepdiv2*InvSqrt(n)
//#define xInvSqrt(n) n>0.25?stepdiv2*InvSqrt(n):2*stepdiv2

float pta = 0;
//#define xInvSqrt(d,n) pta=(pta+n)*0.5;d=pta>0.25?stepdiv2*InvSqrt(pta):2*stepdiv2;
#define xInvSqrt(d,n) d=n>0.25?stepdiv2/(sqrt(n)):2*stepdiv2;
#define xInvVel(d,n) d=stepdiv2/(n);


#define sqrt32(n) sqrt(n)



#ifdef ISPC
void init_home_input() {};
void close_home_input() {};
#else
int home_cnt = 0;
void THEISR home_input() {
  home_cnt++;
  // tap home 3 times to start printing
}
void init_home_input() {
  home_cnt = 0;
#ifdef limit_pin
  pinMode(limit_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limit_pin), home_input, CHANGE);
#endif
}
void close_home_input() {
#ifdef limit_pin
  detachInterrupt(digitalPinToInterrupt(limit_pin));
#endif
}

#endif

/*
  =================================================================================================================================================
  RESET_MOTION
  =================================================================================================================================================
  Reset all variable
*/
// keep last direction to enable backlash if needed
float perstepx,perstepy,perstepz;

void reset_motion()
{

  // 650bytes code
#ifdef SUBPIXELMAX
  vSUBPIXELMAX = SUBPIXELMAX;
#endif
  homingspeed = HOMINGSPEED;
  xycorner = XYCORNER;
  xyjerk = XYJERK;
  zcorner = fmin(homingspeed / 3, xycorner);
  zjerk = xyjerk * zcorner / xycorner;
  Lscale = LSCALE;
  ishoming = 0;
  cmctr = 0;
  e_multiplier = f_multiplier =  1;
  checkendstop = 0;
  endstopstatus = 0;
  xcheckevery = CHECKENDSTOP_EVERY * stepmmx[0];
  ycheckevery = CHECKENDSTOP_EVERY * stepmmx[1];
  zcheckevery = CHECKENDSTOP_EVERY * stepmmx[2];
  echeckevery = CHECKENDSTOP_EVERY * stepmmx[3];
  head = tail = tailok = 0;
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  ocz1 = 0;
  ce01 = 0;
  extadv = 0;
#ifdef ISPC
  tick = 0;
#endif
#ifndef SAVE_RESETMOTION

#ifdef NONLINEAR
  delta_diagonal_rod = DELTA_DIAGONAL_ROD;
  delta_radius = DELTA_RADIUS;
#endif


  maxf[MX] = XMAXFEEDRATE;
  maxf[MY] = YMAXFEEDRATE;
  maxf[MZ] = ZMAXFEEDRATE;
  maxf[nE] = E0MAXFEEDRATE;

  maxa[MX] = XACCELL;
  maxa[MY] = XACCELL * YMAXFEEDRATE / XMAXFEEDRATE;
  maxa[MZ] = XACCELL * ZMAXFEEDRATE / XMAXFEEDRATE;
  maxa[nE] = XACCELL;


  accel = XACCELL;
  zaccel = accel * ZMAXFEEDRATE / XMAXFEEDRATE;


  stepmmx[nE]=fabs(E0STEPPERMM);
  stepmmx[MX]=fabs(XSTEPPERMM);
  stepmmx[MY]=fabs(YSTEPPERMM);
  stepmmx[MZ]=fabs(ZSTEPPERMM);
  perstepx=1.0/(XSTEPPERMM);
  perstepy=1.0/(YSTEPPERMM);
  perstepz=1.0/(ZSTEPPERMM);
  

  odir[MX] = XSTEPPERMM < 0 ? -1 : 1;
  odir[MY] = YSTEPPERMM < 0 ? -1 : 1;
  odir[MZ] = ZSTEPPERMM < 0 ? -1 : 1;
  odir[nE] = E0STEPPERMM < 0 ? -1 : 1;


  ax_home[MX] = XMAX;
  ax_home[MY] = YMAX;
  ax_home[MZ] = ZMAX;

  xback[MX] = MOTOR_X_BACKLASH;
  xback[MY] = MOTOR_Y_BACKLASH;
  xback[MZ] = MOTOR_Z_BACKLASH;
  xback[nE] = MOTOR_E_BACKLASH;
  
  axisofs[MX]= XOFFSET;
  axisofs[MY]= YOFFSET;
  axisofs[MZ]= ZOFFSET;
  axisofs[nE]= EOFFSET;
  

#endif
}

void preparecalc()
{
#ifdef NONLINEAR

  nonlinearprepare();

#endif

}

int32_t mcx[NUMAXIS];
int32_t lctx,lcdx;
/*
  =================================================================================================================================================
  MOTOR CLASS
  =================================================================================================================================================
*/



#define FASTAXIS(n) ((n->status >> 4)&3)
//if(m)coreloop();
#include "motors.h"

int mb_ctr;

void power_off()
{
  motor_0_OFF();
  motor_1_OFF();
  motor_2_OFF();
  motor_3_OFF();

}
/*
  =================================================================================================================================================
  PREPARERAMP
  =================================================================================================================================================
  Bertujuan mengkalkulasi berapa tangga aselerasi awal (rampup) dan akhir (rampdown)
  rampup belum tentu speed naik, bisa jadi speed turun.

  Profil dari gerakan ditentukan oleh fs (fstart), fn (f nominal), fe (f end)
  dengan aselerasi yang ditentukan, maka dihitung berapa rampup, dan down

  Apabila rampup dan down bertemu, atau melebihi jarak totalstep, maka perlu dikalkulasi lagi fs,fn dan fe supaya ramp bisa terpenuhi
  Untuk rampdown, harusnya rekursif ke belakang apabila tidak cukup jarak rampdown. namun implementasi sekarang
  hanya melakukan kalkulasi ulang aselerasi supaya rampdown maksimal ya sama dengan totalstep


  24-4-2018, i change the math of ramp calculation using mm distance


  codes of the prepare ramp are in fcurve.h and scurve.h, they both are constant jerk implementation

* */
float currdis, prevdis, fe;

#ifdef NONLINEAR
float rampv;
float rampseg, rampup, rampdown;
#else
int32_t rampseg, rampup, rampdown;
#define rampv 1
#endif
#define sqr(x) (x)*(x)


/*
  =================================================================================================================================================
  COMMAND BUFFER
  =================================================================================================================================================
  Store real motor motion and the timing. this will decouple the step generator from Timer interrupt.
  Step generator can run at any idle time, filling the command buffer.

  Command buffer is small only 4byte
  Bit 0: CMD type 0:header 1:motion

  Header
  Bit 1,2,3,4 AXIS direction
  Bit 5,6,7,8 AXIS is move
  bit 9-17 Laser Intensity, 0 is off, 255, is MAX

  Motion
  Bit 1,2,3,4 Axis step
  Bit 5 ... 31 Timing

  total 32 bit, i think for AVR 16 bit is enough ? to save ram

*/
#ifdef __AVR__
#define NUMCMDBUF 255
#else
#define NUMCMDBUF 1023
#endif


//#define nextbuffm(x) ((x) < NUMCMDBUF ? (x) + 1 : 0)
#define nextbuffm(x) ((x+1)&NUMCMDBUF)
#define nextbuffmV(x,v) ((x+v)&NUMCMDBUF)

static volatile uint32_t cmddelay[(NUMCMDBUF+1)], cmd0;
static volatile uint8_t   cmcmd, cmbit = 0;
static volatile uint32_t cmhead = 0, cmtail = 0, cmdlaserval = 0;

#define cmdfull (nextbuffm(cmhead)==cmtail)
#define cmdnotfull (nextbuffm(cmhead)!=cmtail)
#define cmdempty (cmhead==cmtail)
#define cmdlen (cmhead >= cmtail ? cmhead - cmtail : ((NUMCMDBUF+1) + cmhead) - cmtail)

static volatile int nextok = 0, laserwason = 0;

int sendwait = 0; int delaywait = 1;

// we must interpolate delay here, between 10 steps
int ldelay = 0;


static void pushcmd()
{
  // no more data if endstop hit
  if (checkendstop && (endstopstatus < 0))return;
  // wait until a buffer freed

  while (cmdfull) {
#ifdef USETIMER1
    MEMORY_BARRIER();
#else
    CORELOOP
#endif
  }

  // if move cmd, and no motor move, save the delay
  // this only happen when subpixel accuracy is enabled
  // need to test it
  if ( (cmd0 & 1) && !(cmd0 & (15 << 1))) {
    ldelay += cmd0 >> 5;
  } else {
    cmhead = nextbuffm(cmhead);
    //if (cmd0 & 1)cmd0 += (ldelay << 5);
    cmddelay[cmhead] = cmd0;
    //ldelay = 0;
  }

}
void newdircommand(int laserval)
{
  // change dir command
  //cmd0 = 0;//DIRDELAY << 6;
  cmd0 = (laserval << 9);
  //zprintf(PSTR("int %d\n"), fi(laserval));
  if (sx[0] > 0)cmd0 |= 2;
  if (sx[1] > 0)cmd0 |= 8;
  if (sx[2] > 0)cmd0 |= 32;
  if (sx[3] > 0)cmd0 |= 128;
  // TO know whether axis is moving
  if (bsdx[0] > 0)cmd0 |= 4;
  if (bsdx[1] > 0)cmd0 |= 16;
  if (bsdx[2] > 0)cmd0 |= 64;
  if (bsdx[3] > 0)cmd0 |= 256;
  ldelay = 0;
  pushcmd();
  cmd0 = stepdivL;
  pushcmd();

}



#define bresenham(ix)\
  if ((mcx[ix] -= bsdx[ix]) < 0) {\
    cmd0 |=2<<ix;\
    mcx[ix] += totalstep;\
  }


/*
	-------------------------------------------- BEGIN OF SCURVE IMPLEMENTATION ----------------------------------------------
*/

uint8_t cntF;
int prevacc;
int nextacc;
int curracc;
int scurve, has1, has2, has3, has4, has5, has6, has7;
int sg, ok;
int lsteps;

float s1, s2, s3, s4, s5, s6, s7;
float va, vi, vc, ve, vjerk1, vjerk7, mvjerk;


#ifdef TRUESCURVE
#include "scurve.h"
#else
#include "fcurve.h"
#endif

/*
	-------------------------------------------- END OF SCURVE IMPLEMENTATION ----------------------------------------------
*/


/*
  =================================================================================================================================================
  PLANNER
  =================================================================================================================================================
  dipanggil oleh   untuk mulai menghitung dan merencakanan gerakan terbaik
  kontrol mbelok adalah cara mengontrol supaya saat menikung, kecepatan dikurangi sehingga tidak skip motornya

*/
float curru[5], prevu[5];
float lastf = 0;

void canceldecelerate(int m) {

}
void backforward()
{
  int h;
  tmove *next, *curr;
  // backward from last moves (head) to the last known planned (tailok)
  h = head;
  if (h == tailok) return;
  curr = 0;
  while (1) {
    next = curr;
    // if this the last item then exit speed fes is MINCORNERSPEED, perhaps almost 0
    // if not then exit speed is next entry speed (->fs)
    float fes = next ? next->fs : MINCORNERSPEED;
    curr = &moves[h];
    // if curr entry speed is not equal maximum junction allowable speed
    if (curr->fs != curr->maxs) {
      // change the entry speed to which one is minimum
      //  - maximum junction speed
      // or delta from acceleration fron exit speed
      curr->fs = fmin(fes + curr->delta, curr->maxs); // maximum speed from next start
    }
    // move back the pointer
    h = prevbuff(h);
    // if arrive at planned ok then break
    if (h == tailok)break;
  }


  // forward from the last planned, i think h is already set from previous loop
  // h = tailok;

  next = &moves[h];
  // next pointer
  h = nextbuff(h);
  // loop while h still not arrive head
  while (h != head) {
    curr = next;
    next = &moves[h];
    // if current start speed is less than next entry speed
    if (curr->fs < next->fs) {
      float fs = curr->fs + curr->delta; // maximum speed at end of current move
      // if next move entry speed is faster then make it lower to this speed
      if (fs < next->fs) {
        next->fs = fs;
        // and make this curr move is planned ok
        // current move is previous h, because h point to next move
        tailok = h;//prevbuff(h);
      }
    }
    // also if next entry speed is equal the maximum junction speed then make it planned ok
    if (next->fs == next->maxs) tailok = h;//prevbuff(h);
    // next pointer
    h = nextbuff(h);
  }
  // call motion loop (for non hardware timer )
  CORELOOP
}

float mmdis[4];
void planner(int32_t h)
{
  // mengubah semua cross feedrate biar optimal secepat mungkin
  tmove *curr;
  // point to current moves
  curr = &moves[h];
  // copy the last current vector (curru) to previous (prevu)
  // we want to update the current (curru)
  memcpy(prevu, curru, sizeof prevu);
  //curru[4] = 1; // ???
  // set the large first, we want to get the minimum acceleration and speed
  float ac = accel;
  float fn = curr->fn;
  for (int i = 0; i < 4; i++) {
    curru[i] = 0;
    if (curr->dx[i] != 0) {
      // vector at axis i is distance of the axis / distance total
      curru[i] = float(mmdis[i] / curr->dis);
      // real accel and speed each axis, we want to know the maximum value of each axis, and get
      // the minimum from all axis , so no axis will move more than the maximum value configured
      ac = fmin(ac, fabs(maxa[i] / curru[i]));
      fn = fmin(fn, fabs(maxf[i] / curru[i]));
      CORELOOP
    }
  }
  // xycorner is value configured at eeprom
  float ucorner = xycorner * ac * 0.001;
  // calculate and save the delta speed from this acceleration
  curr->ac = 2 * ac;
  curr->delta = fabs(curr->ac * curr->dis);
  // current speed is the minimum
  curr->fn = sqr(fn);
  /**  
  // ==================================
  // Centripetal corner max speed calculation, copy from other firmware
  float MINSPEED = MINCORNERSPEED; //pow(xyjerk,0.06667);//
  float max_f = MINSPEED;
  if (bufflen > 1) {
    // if already more than 1 vector, lets calculate the maximum junction speed using
    // centripetal formula
    float junc_cos = -prevu[MX] * curru[MX] - prevu[MY] * curru[MY] - prevu[MZ] * curru[MZ];
    if (junc_cos > 0.99999) {
      max_f = MINSPEED; // reversal case, angle is 180'
    } else if (junc_cos < -0.99999) {
      max_f = 10000000; // straight line, set a large value
    } else {
      // using cheap mathematical formula, check GRBL for the full code
      float sin_theta_d2 = sqrt(0.5 * (1 - junc_cos)); // Trig half angle identity. Always positive.
      max_f = fmax( MINSPEED, (ucorner * sin_theta_d2) / (1.0 - sin_theta_d2) );
    }

    CORELOOP

  }
  // set the max junction speed
  // its cannot greater than both end of the moves (lastf and curr->fn)
  curr->maxs = fmin(
                 (fmin(curr->fn, lastf)),
                 max_f);
  lastf = curr->fn;
  **/               
  curr->maxs=fmin(curr->fn,lastf);
  lastf=curr->fn;
  if (bufflen>1) {
    float junc_cos = -prevu[MX] * curru[MX] - prevu[MY] * curru[MY] - prevu[MZ] * curru[MZ];
    if (junc_cos*junc_cos < 0.9999) {
      float sin_theta_d2 = sqrt(0.5 * (1.0 - junc_cos)); // Trig half angle identity. Always positive.
      curr->maxs=fmin(curr->maxs,0.5+(ucorner * ucorner* sin_theta_d2 / (1.0 - sin_theta_d2) ));
    }
  }
    
                 
#ifdef output_enable
  zprintf(PSTR("maxs. %f\n"), ff(curr->maxs));
#endif
  // do backward and forward planning
  backforward();
}

/*
  =================================================================================================================================================
  ADDMOVE
  =================================================================================================================================================
  add path to motion system
*/
int32_t x1[NUMAXIS], x2[NUMAXIS];
tmove *m;

#ifdef ISPC
float fctr, tick;
float tickscale, fscale, graphscale;
#endif

int32_t  f;
float ta, oacup, oacdn;
int32_t mctr, xtotalseg;
uint8_t fastaxis;
uint32_t nextmicros;
uint32_t nextdly;
uint32_t nextmotoroff;
float otx[NUMAXIS]; // keep the original coordinate before transform
int8_t repos = 0;

#ifdef USE_BACKLASH
int ldir[NUMAXIS] = {0, 0, 0, 0};
#endif



void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0 = 1, int rel = 0)
{

  // check if the buffer is full and wait
  needbuffer();

  int32_t x2[NUMAXIS];
#ifdef output_enable
  zprintf(PSTR("\n\nADDMOVE\nTail:%d Head:%d \n"), fi(tail), fi(head));
  zprintf(PSTR("F:%f From X:%f Y:%f Z:%f E:%f\n"), ff(cf), ff(cx1), ff(cy1), ff(cz1), ff(ce01));
  zprintf(PSTR("To X:%f Y:%f Z:%f E:%f\n"),  ff(cx2), ff(cy2), ff(cz2), ff(ce02));
#endif
  tmove *curr;
  curr = &moves[nextbuff(head)];
  curr->status = g0 ? 8 : 0; // set the motion type G0 (hi) or G1 (lo) bit to 3rd bit
  // babystep store values in INTEGER 1000x original value so we restore the value here
  if (babystep[0])cx1 -= babystep[0] * 0.001;
  if (babystep[1])cy1 -= babystep[1] * 0.001;
  if (babystep[2])cz1 -= babystep[2] * 0.001;
  if (babystep[3])ce01 -= babystep[3] * 0.001;
  // if motion is relative then add
  if (rel) {
    cx2 += cx1;
    cy2 += cy1;
    cz2 += ocz1;
    ce02 += ce01;
  }
  if (!ishoming) {
    // limit movement to prevent damaged gantry part  
    //if (cx2<-3)cx2=-3;
    //if (cy2<-3)cy2=-3;
  }

  // mesh leveling
  // get Z offset on mesh data
  float ocz2 = cz2;
  if (MESHLEVELING) {
    cz2 -= Interpolizer(cx2, cy2);
  }

#ifdef ISPC
  curr->col = 2 + (head % 4) * 4;
#endif
  // this x2 is local

  // calculate new distance each axis, in mm
  mmdis[0] = (cx2 - cx1) * odir[0];
  mmdis[1] = (cy2 - cy1) * odir[1];
  mmdis[2] = (cz2 - cz1) * odir[2];
  mmdis[3] = (ce02 - ce01) * odir[3];

#ifdef output_enable
  zprintf(PSTR("Dis X:%f Y:%f Z:%f\n"),  ff(mmdis[0]), ff(mmdis[1]), ff(mmdis[2]));
#endif

  for (int i = 0; i < NUMAXIS; i++) {
    x2[i] = 0;
#define deltax mmdis[i]


#ifdef USE_BACKLASH
    int dir;
    if (deltax < 0)dir = -1;
    else if (deltax > 0)dir = 1;
    else dir = 0;

    // if there is movement and have save last dir, and last dir <> current dir then add backlash
    if ((ldir[i] != 0) && (dir != 0) && (ldir[i] != dir)) {
      // add backlash steps to this axis
      // backlash data is INTEGER 1000x of original value so we need to * 0.001
      float b = xback[i] * dir * 0.001;
      deltax += b;
      x2[i] = b * Cstepmmx(i) * odir[i];
    }
    // if no movement, then dont save direction
    if (dir != 0)ldir[i] = dir;
#endif
  }

  // Calculate the real distance of the path, its depends on the DRIVE MODE
#if defined( DRIVE_COREXY)
  curr->dis = sqrt(sqr2(mmdis[0] + mmdis[1]) + sqr2(mmdis[1] - mmdis[0]) + sqr2(mmdis[2]) + (sqr2(mmdis[3])));
#elif defined( DRIVE_COREXZ)
  curr->dis = sqrt(sqr2(mmdis[0] + mmdis[2]) + sqr2(mmdis[2] - mmdis[0]) + sqr2(mmdis[1]) + (sqr2(mmdis[3])));
#else
  curr->dis = sqrt(sqr2(mmdis[0]) + sqr2(mmdis[1]) + sqr2(mmdis[2]) + (sqr2(mmdis[3])));
#endif


#ifndef NONLINEAR
  // for LINEAR, just keep the INT value here
  x2[0] += (((int32_t)(cx2 * Cstepmmx(0)) - (int32_t)(cx1 * Cstepmmx(0))) ) ;
  x2[1] += (((int32_t)(cy2 * Cstepmmx(1)) - (int32_t)(cy1 * Cstepmmx(1))) ) ;
  x2[2] += (((int32_t)(cz2 * Cstepmmx(2)) - (int32_t)(cz1 * Cstepmmx(2))) ) ;


#else
  // for NONLINEAR we keep the floating point
  x2[0] += (cx2 - cx1) * Cstepmmx(0);
  x2[1] += (cy2 - cy1) * Cstepmmx(1);
  x2[2] += (cz2 - cz1) * Cstepmmx(2);
#endif




  CORELOOP

  x2[3] = (int32_t)(ce02 * e_multiplier * Cstepmmx(3)) - (int32_t)(ce01 * e_multiplier * Cstepmmx(3));

#if defined( DRIVE_COREXY)
  // 2 = z axis + xy H-gantry (x_motor = x+y, y_motor = y-x)
  // borrow cx1,cy1 for calculation
  int32_t da = x2[0] + x2[1];
  int32_t db = x2[1] - x2[0];
  x2[0] = da;
  x2[1] = db;

#elif defined(DRIVE_COREXZ)
  // 8 = y axis + xz H-gantry (x_motor = x+z, z_motor = x-z)
  int32_t da = x2[0] + x2[2];
  int32_t db = x2[0] - x2[2];
  x2[0] = da;
  x2[2] = db;
#endif

#if defined(DRIVE_XYYZ)
  // copy dZ to dE and  dY to dZ, for CNC with 2 separate Y axis at homing.
  if (ishoming) {
    // when homing the z will threat as normal cartesian Z that can be homing normally.
  } else {
    //
    x2[3] = x2[2];
    x2[2] = x2[1];
  }
#elif defined(NONLINEAR)
  // nothing to do
#endif
  // retract conversion
  if (cf <= 1) {
    // retract in
    if (x2[3] < 0) {
      x2[3] = -retract_in * Cstepmmx(3);
      cf = retract_in_f;
    }
    // retract in
    if (x2[3] > 0) {
      x2[3] = retract_out * Cstepmmx(3);
      cf = retract_out_f;
    }
  }

  // if no axis movement then dont multiply by multiplier
  if (g0 || ishoming || ((x2[0] == 0) && (x2[1] == 0) && (x2[2] == 0)) ) {} else cf *= f_multiplier;
#ifdef __AVR__
  if (cf > 200)cf = 200; // prevent max speed
#endif

  CORELOOP
  // Now we can fill the current move data
  curr->fn = cf;
  curr->fs = 0;
  int32_t dd = 0;
  int faxis = 0;

#ifdef SHARE_EZ
  // On Wemos 3D printer, its not enough Direction pin, so we can use SHARE E and Z, and we will skip the E motion when Z motion
  // present
  // if delta[nZ] is not same direction with delta[nE] then
  //  if (cz2 != cz1) ce02 = ce01; // zeroing the E movement if Z is moving and ZE share direction pin
#define delt(i) x2[i]*odir[i]
  if ((delt(2) != 0) && (delt(2)*delt(3) < 0)) {
    ce02 = ce01; // zeroing the E movement if Z is moving and ZE share direction pin
    x2[3] = 0;
  }
#endif

  // find the largest axis steps
  for (int i = 0; i < NUMAXIS; i++) {
    babystep[i] = 0; // zeroing the babystep
    int32_t delta = x2[i];
    curr->dx[i] = delta;
    delta = abs(delta);
    if (delta > dd) {
      dd = delta;
      faxis = i;
    }
  }

  CORELOOP
  // if zero length then cancel
  if (dd > MINSTEP) {
#ifdef output_enable
    zprintf(PSTR("Totalstep AX%d %d\n"), (int32_t)faxis, (int32_t)curr->dx[faxis]);
#endif
    curr->status |= faxis << 4;
    //zprintf(PSTR("F:%f A:%d\n"), ff(cf), fi(curr->ac));
    if (head == tail) { // if this the first entry in buffer then keep the data on OTX
      otx[0] = cx1;
      otx[1] = cy1;
      otx[2] = cz1;
      otx[3] = ce01;
      curr->status |= 128;
    }

    // Laser

    curr->laserval = (laserOn ? constantlaserVal : 0);
    head = nextbuff(head);
    curr->status |= 1; // 0: finish 1:ready
    // planner are based on cartesian coord movement on the motor
    planner(head);
    //#ifdef NONLINEAR
    // we keep the original target coordinate, to get correct stop position when a HARDSTOP performed
    cx1 = curr->dtx[0] = cx2; // save the target, not the original
    cy1 = curr->dtx[1] = cy2;
    cz1 = curr->dtx[2] = cz2;
    ocz1 = ocz2;
    ce01 = curr->dtx[3] = ce02;

  }

}





#define N_ARC_CORRECTION 25
#ifdef ARC_SUPPORT

/*

  ARC using Float implementation

*/

uint32_t approx_distance(uint32_t dx, uint32_t dy)
{
  uint32_t min, max, approx;

  // If either axis is zero, return the other one.
  if (dx == 0 || dy == 0) return dx + dy;

  if ( dx < dy ) {
    min = dx;
    max = dy;
  } else {
    min = dy;
    max = dx;
  }

  approx = ( max * 1007 ) + ( min * 441 );
  if ( max < ( min << 4 ))
    approx -= ( max * 40 );

  // add 512 for proper rounding
  return (( approx + 512 ) >> 10 );
}

void draw_arc(float cf, float cx2, float cy2, float cz2, float ce02, float fI, float fJ, uint8_t isclockwise)
{
#define debug0
  uint32_t radius = approx_distance(abs(fI), abs(fJ));
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc

  float cx = cx1 + fI ;
  float cy = cy1 + fJ ;
#ifdef debug1
  zprintf(PSTR("Arc  I, J,R:%f,%f,%d\n"), fI, fJ, radius);
  zprintf(PSTR("go cX cY:%f %f\n"), cx, cy);
#endif

  //uint32_t linear_travel = 0; //target[axis_linear] - position[axis_linear];
  float ne = ce01;
  float extruder_travel = (ce02 - ce01);
  float nz = ocz1;
  float z_travel = (cz2 - ocz1);

  float r_axis0 = -fI;  // Radius vector from center to current location
  float r_axis1 = -fJ;
  float rt_axis0 = cx2 - cx;
  float rt_axis1 = cy2 - cy;

#ifdef debug1
  sersendf_P(PSTR("go rX rY:%f %f\n"), r_axis0, r_axis1);
  sersendf_P(PSTR("go rtX rtY:%f %f\n"), rt_axis0, rt_axis1);
#endif

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001)) {
    angular_travel += 2.0f * M_PI;
  }
  if (isclockwise) {
    angular_travel -= 2.0f * M_PI;
  }

  uint32_t millimeters_of_travel = abs(angular_travel * radius); //hypot(angular_travel*radius, fabs(linear_travel));
#ifdef debug1
  sersendf_P(PSTR("go ang mm:%f %d\n"), angular_travel, millimeters_of_travel);
#endif
  if (millimeters_of_travel < 1) {
    return;// treat as succes because there is nothing to do;
  }
  int32_t segments = fmax(1, millimeters_of_travel * 10);

  float theta_per_segment = angular_travel / segments;
  float extruder_per_segment = (extruder_travel) / segments;
  float z_per_segment = (z_travel) / segments;

  float cos_T = 2.0 - theta_per_segment * theta_per_segment;
  float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
  cos_T *= 0.5;

  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  int32_t i;
  int32_t count = 0;

  // Initialize the linear axis
  //arc_target[axis_linear] = position[axis_linear];

  // Initialize the extruder axis


  for (i = 1; i < segments; i++) {
    // Increment (segments-1)

    if (count < N_ARC_CORRECTION) { //25 pieces
      // Apply vector rotation matrix
      r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
      r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti  = cos(i * theta_per_segment);
      sin_Ti  = sin(i * theta_per_segment);
      r_axis0 = -fI * cos_Ti + fJ * sin_Ti;
      r_axis1 = -fI * sin_Ti - fJ * cos_Ti;
      count = 0;
    }

    // Update arc_target location
    float nx = cx + r_axis0;
    float ny = cy + r_axis1;
    ne += extruder_per_segment;
    nz += z_per_segment;
    //move.axis[nE] = extscaled>>4;
    addmove(cf, nx, ny, nz, ne, 0, 0);
  }
  // Ensure last segment arrives at target location.
  addmove(cf, cx2, cy2, cz2, ce02, 0, 0);
}
#endif



#ifdef NONLINEAR
void calculate_delta_steps();
#endif


static uint32_t cmdly;
int cornerctr = 0;

#ifdef ISPC
int laseron = 0;
#define LASER(x) {laseron=x==LASERON;}

float xs[4] = {0, 0, 0, 0};
float gx, gy, lx, ly;
int pcsx[4];
#define graphstep(ix) xs[ix] +=pcsx[ix]
float lstepdiv = 1;
void dographics();
#endif

/*
  =================================================================================================================================================
  MOTIONLOOP
  =================================================================================================================================================
*/
void otherloop(int r);
uint32_t cm, ocm,  mctr2, dlmin, dlmax, cmd_mul;
int32_t timing = 0;
int32_t laser_step;


// ======================================= COMMAND BUFFER ===========================================
int maincmdlaserval = 0;
int32_t laser_accum;
uint32_t laser_mul;
static THEISR void decodecmd()
{
  if (cmdempty) {
    RUNNING = 1;
    LASER(!LASERON);
    return;
  }

  // if on pause we need to decelerate until command buffer empty

  // to make sure the overrides speed is gradual, not sudden change
  //if (f_multiplier > f_rmultiplier)f_rmultiplier += 0.005;
  //else if (f_multiplier < f_rmultiplier)f_rmultiplier -= 0.005;


  uint32_t cmd = cmddelay[cmtail];
  // cmcmd is the 1st bit of the command
  cmcmd = cmd & 1;
  // SLOWDOWNING is perform slowing down if the command buffer count is low
#define SLOWDOWNING

#ifdef SLOWDOWNING

  if ((cmtail & 15) == 0) {
    //
    cmd_mul = 0;
    int bl = cmdlen;

    // every 15 command, slowdown if buffer is low
    if (bl < (NUMCMDBUF / 6)) cmd_mul = 1;
    else if (bl < (NUMCMDBUF / 5)) cmd_mul = 2;
    else if (bl < (NUMCMDBUF / 4)) cmd_mul = 4;
    else if (bl < (NUMCMDBUF / 2)) cmd_mul = 5;

  }
#endif

  if (cmcmd) {
    // 1st bit is set then this is the motion command
    cmbit = (cmd >> 1) & 15;
    // if nothing to move then turn off laser , its end of the move
    // inform if non move is in the buffer
    //if (cmcmd && (cmbit==0))zprintf(PSTR("X"));
    cmdly = (cmd >> 6);
  } else {
    // this is motion header command, contain the motor direction and the laser on/off
    cmbit = (cmd >> 1) & 255;
    cmdly = DIRDELAY;


    cmdlaserval = maincmdlaserval = (cmd >> 9) & 255;
    laserwason = (Setpoint == 0) && (maincmdlaserval > 0);
    laser_step = 100000;
    laser_accum=0;
    cmtail = nextbuffm(cmtail);
    laser_mul = cmddelay[cmtail];
    if (laserwason) {
      if ((cmdlaserval < 255)) { //if less than 255 mean laser on for defined time
        // laser step is the laser delay ( constant burn laser)
        // the delay can be adjusted by GCODE Svalue, and can be calibrated using
        // eeprom Lscale
        laser_step = cmdlaserval * laser_mul; // 254
        if (cmdlaserval && (laser_step<10))laser_step=10;
      }
    }
  }
  // calculate the total motion delay
  cmdly = (cmdly
#ifdef SLOWDOWNING
           + (cmd_mul > 0 ? (cmdly << 1) >> (cmd_mul) : 0)
#endif
          ) >> DSCALE;


  // signal this command is OK
  nextok = 1;
  // point next command
  cmtail = nextbuffm(cmtail);

  // set the timer, non hardware timer need to work on constant laserburn
  // but since, all hardware have use hardwaretimer, i think maybe its
  // good time to remove the non hardware timer code, and clean up the source code
  //laser_step=1;
  // lets do dithering here
  
  if (cmcmd){
      if (laser_accum>-1000000) laser_accum-=laser_step;
      if (laser_accum<0){
          LASER(laserwason? LASERON : !LASERON);
          laser_accum+=cmdly;
      } else {
          LASER(!LASERON);
      }
  }
  timer_set2(cmdly,0);
#ifdef heater_pin
  HEATER(HEATING);
#endif

}

uint32_t mc, dmc, cmctr;
int32_t e_ctr = 0;
int e_dir,x_dir,y_dir,z_dir;
int mm_ctr = 0;
int32_t info_x_s,info_y_s,info_z_s;

void THEISR coreloopm()
{
  //dmc=(micros()-mc); mc=micros();
  if (!nextok) {
    decodecmd();
    return;
  }

#ifdef USETIMER1
  {
#elif defined(ISPC)
  {
#else
  // non hardware timer code

  cm = micros();
  if (cm - nextmicros >= ndelay) {
    nextmicros = cm;
    if (ndelay2) {
      // turn off laser , laser constant burn mode
      ndelay = ndelay2;
      ndelay2 = 0;
      LASER(!LASERON);
      return;
    }
#endif
    // execute motion command
    if (cmcmd) { // 1: move
      mm_ctr++; // this counter used by probing to calculate real mm

      // AXIS 4 - Extruder
      if (cmbit & 8) {
#ifndef DRIVE_XZY2
        ectstep += e_dir;
        motor_3_STEP();
#endif
      };
      // AXIS 1 - X
      if (cmbit & 1) {
        //cmctr++;
        xctstep++;
        info_x_s-=x_dir;    
        motor_0_STEP();
      }
      // AXIS 2
      if (cmbit & 2) {
        yctstep++;
        info_y_s-=y_dir;    
        motor_1_STEP();
        #ifdef COPY_Y_TO_Z
        motor_2_STEP();
        #endif
#ifdef DRIVE_XZY2
        // copy motion on motor E
        motor_3_STEP();
#endif
      }
      // AXIS 3
      if (cmbit & 4) {
        #ifdef COPY_Y_TO_Z
        #else
        zctstep++;
        info_z_s-=z_dir;    
        motor_2_STEP();
        #endif
      }
      pinCommit();
      // after commit the PIN, send the LOW signal.
      // pincommit are used for shift register based pin, and its not used by real pin
      motor_3_UNSTEP();
      motor_0_UNSTEP();
      motor_1_UNSTEP();
      motor_2_UNSTEP();
      pinCommit();

#ifdef ISPC
      if (cmbit & 1)graphstep(0);
      if (cmbit & 2)graphstep(1);
      if (cmbit & 4)graphstep(2);
      if (cmbit & 8)graphstep(3);
      dographics();
#endif
      // Endstop check routine
      if (checkendstop) { // check endstop every 30 step
        if ((xctstep >= xcheckevery) || (yctstep >= ycheckevery) || (zctstep >= zcheckevery)) { // || (ectstep >= echeckevery) ) {
          xctstep = yctstep = zctstep = ectstep = 0;
          docheckendstop(0);
          if (endstopstatus != 0) {
            cmtail = cmhead;
            nextok = 0;
            return; // no more move and clear cmdbuffer if endstop hit/
          }
        }
      }

    } else { // 0: set dir
      // header command, we set the motor driver direction
      e_dir = 0;
      if (cmbit & 2) motor_0_DIR(x_dir=(cmbit & 1) ? -1 : 1);
      if (cmbit & 32) motor_2_DIR(z_dir=(cmbit & 16) ? -1 : 1);
      if (cmbit & 128) {
#ifndef DRIVE_XZY2
        
        motor_3_DIR(e_dir = (cmbit & 64) ? -1 : 1);
        //zprintf(PSTR("E:%d\n"), fi(e_dir));
#endif
      }
      if (cmbit & 8) {
          int d1=(y_dir=(cmbit & 4)) ? -1 : 1;
          motor_1_DIR(d1);
          #ifdef COPY_Y_TO_Z
          motor_2_DIR(odir[2]*d1);
          #endif
      }  

      pinCommit();


#ifdef ISPC
      pcsx[0] = (cmbit & 1) ? 1 : -1;
      pcsx[1] = (cmbit & 4) ? 1 : -1;
      pcsx[2] = (cmbit & 16) ? 1 : -1;
      pcsx[3] = (cmbit & 64) ? 1 : -1;
#endif
    }
    // next command
    nextok = 0;
    decodecmd();
  }
}


/* ================================================================================================================
  PC SIMULATION

  this need to move to interrupt for smooth movement
  ================================================================================================================
*/

#ifdef ISPC
void dographics()
{
  if (cmdly < DIRDELAY) {
    zprintf(PSTR("Odd delay:%d\n"), cmdly);
    return;
  }
  // this might be wrong because the last cmd maybe is from previous segment which have different step/mm
  float f = CLOCKCONSTANT / cmdly;
  lstepdiv = stepdiv;
#ifdef TRUESCURVE
  //f = (V);//sqrt(ta);
#else
  //f = sqrt(V);//sqrt(ta);
#endif
  //f =  sqrt(ta)/stepdiv2;
  tick = tick + cmdly; //1.0/timescale;
  int32_t c;
  // color by segment
  c = (tail & 3) + 10;
  if (cornerctr > 0)c = 1;
  // color by speed
  //c = (f - 20) * 4;

  //float cf = float(timescale) / dl;



  //pset (tick*tickscale+10,400-f*fscale),c
#define realpos1(i) (xs[i] / stepmmx[i])
  if (tick * tickscale / timescale > 600) {
    gx = 0;
    tick = 0;
  }
  //putpixel (tick * tickscale / timescale + 30, 450 -  fscale * cf, c - 8);
  lx = gx;
  ly = gy;
  gx = tick * tickscale / timescale + 10;
  gy = 480 -  fscale * f * 0.01;

  setcolor(1);
  putpixel (gx - 1, 480, 1);
  setcolor(2);
  putpixel (gx - 1, 480 - 50 * fscale, 2);
  setcolor(3);
  putpixel (gx - 1, 480 - 100 * fscale, 3);

  setcolor(c);
  putpixel (gx, gy, c);

  setcolor(1);
  f = V;//sqrt(ta);
  gy = 480 -  fscale * f * 0.1;
  //putpixel (gx, gy, c);

  //line(lx, ly, gx, gy);
  //zprintf(PSTR("X:%f Y:%f\n"), ff(gx), ff(gy));
  setcolor(0);
  line(gx + 1, 480 - 1, gx + 1, 480 - 100);
  gy = 480 -  fscale * 100 * 0.1;
  setcolor(1);
  line(0, gy, 200, gy);

#ifdef NONLINEAR
  float ex = realpos1(0) * graphscale;
  float ey = realpos1(1) * graphscale;
  float ez = realpos1(2) * graphscale;

#if defined( DRIVE_DELTASIAN)
  circle  (150, 251 - ex + 0, 5);
  circle  (350, 251 - ez + 0, 5);
  circle  (251 + ey * 0.1, 251 + ey * 0.1 + 0, 5);
  circle  (150, 249 - ex + 0, 5);
  circle  (350, 249 - ez + 0, 5);
  circle  (249 + ey * 0.1, 249 + ey * 0.1 + 0, 5);
  setcolor(9);
  circle  (150, 250 - ex + 0, 5);
  setcolor(12);
  circle  (350, 250 - ez + 0, 5);
  setcolor(14);
  circle (250 + ey * 0.1, 250 + ey * 0.1 + 0, 5);
#elif defined( DRIVE_DELTA)
  circle(150, 251 + ex + 0, 5);
  circle (350, 251 + ey + 0, 5);
  circle (250, 201 + ez + 0, 5);
  circle (150, 249 + ex + 0, 5);
  circle (350, 249 + ey + 0, 5);
  circle (250, 199 + ez + 0, 5);

  setcolor(9);
  circle (150, 250 + ex + 0, 5);
  setcolor(12);
  circle (350, 250 + ey + 0, 5);
  setcolor(14);
  circle (250, 200 + ez + 0, 5);
#endif

#else
  float ex = realpos1(0) * graphscale;
  float ey = realpos1(1) * graphscale;
  float ez = realpos1(2) * graphscale;

  //putpixel (ex + 320, ey + 150, c);
  putpixel (ex + ey * 0.3 + 120, ey * 0.8 - ez + 150, c);
  //putpixel (ex + 320, ey  + 150, c);
#endif

#ifdef showlaser
  if (laseron) {
    putpixel (ex * 2 + 50 , ey * 2 + 50, 15);
  }// else     putpixel (ex * 5 , ey * 5, 9);
#endif

}
#else
#define graphstep(ix)
#endif

float ia = 0;
long firstdown;

int pixelon = 0;
void readpixel2() {

  //e_ctr=e_ctr&8191;
  char vv = g_str[e_ctr];
  vv &= ~32;
  if (vv == 'A')pixelon = 0; else pixelon = 1;
}

void doHardStop(){
      if (!RUNNING) {
#ifdef HARDSTOP
        float p = mctr;
        cx1 = info_x_s*perstepx;
        cy1 = info_y_s*perstepy;
        cz1 = info_z_s*perstepz;
        ce01 = 0;//m->dtx[3] - p * m->dx[3] / Cstepmmx(3);
        //zprintf(PSTR("Stopped!BUFF:%d\n"), fi(bufflen));
#endif
      }
      endstopstatus = 0;
      checkendstop = 0;
      m->status = 0;
      mctr = 0;
      m = 0;
      head = tail;
      cmhead = cmtail = 0;
}

int coreloop1()
{

#ifdef output_enable
  //if (mctr == 10)zprintf(PSTR("DLY:%d \n"), fi(cmdly));
#else
  //if(mctr==10)zprintf(PSTR("DLY:%d \n"), fi(cmdly));
#endif

  if (!m || (mctr <= 0)) {
    //zprintf(PSTR("R\n"));
    return 0;
  }
  if (!coreloopscurve())return 0;

  CORELOOP
#ifndef ISPC
  //if (mctr % 10 == 0)zprintf(PSTR("%d "), fi(mctr));
  //if (mctr % 4 == 0) zprintf(PSTR("F:%d "), fi(stepdiv/dl));
  //if (mctr2++ % 20 == 0) Serial.println("");
#endif
  //    if (mctr % 10 == 0)zprintf(PSTR("endstp %d\n"),fi(checkendstop));
  if (checkendstop || (!RUNNING)) {
    //      docheckendstop();
    //zprintf(PSTR("%d \n"),fi(mctr));
    if ((endstopstatus < 0) || (!RUNNING)) {
      //zprintf(PSTR("Endstop hit"));
        // need to calculate at least correct next start position based on
        // mctr
        doHardStop();
      return 0;
    }
  }
  return 1;
}

int busy = 0;
// ===============================================
int cctr = 0;
int motionloop()
{
  if (cctr++ > 100000) {
    cctr = 0;
    //Serial.println(ndelay);
  }
  if (PAUSE)return 0;
  if (busy ) {
    zprintf(PSTR("Busy\n"));
    return 0;
  }
  busy = 1;
  int  r;
#ifdef FASTBUFFERFILL2
  for (int ff = FASTBUFFERFILL2 + 1; ff; ff--)
#endif
  {
    CORELOOP
    if (!m ) {
        r = startmove();
    } else {
      //zprintf(PSTR("->%d\n"),fi(mctr));

      r = coreloop1();
      //zprintf(PSTR("<-%d\n"),fi(mctr));
      //if (mctr > 0)
    }
  }  // for auto start motion when idle
  otherloop(r);
  servo_loop();
  busy = 0;
  return r;
}
long last_c0 = 0;

bool inwaitbuffer;

float info_x, info_y, info_z, info_e;
void otherloop(int r)
{
  cm = micros();
  if ((cm - last_c0 > 20000)) { // update every 20ms
    last_c0 = cm;
#ifdef HARDSTOP
    info_x = info_x_s*perstepx;
    info_y = info_y_s*perstepy;
    info_z = info_z_s*perstepz;
    extern void setXYZservo(float x,float y,float z);
    setXYZservo(info_x,info_y,info_z);
#endif
  }
  // NON TIMER
  //    zprintf(PSTR("%d\n"),fi(dmc));
  if (m) {
    nextmotoroff = cm;
    if (mctr == 0) {


#ifdef NONLINEAR
      // delta need to check if totalseg
      if (xtotalseg > 0) {
        // next segment
        // calculate next bresenham
        calculate_delta_steps();
      } else
#endif //delta
      {

        // coreloop ended, check if there is still move ahead in the buffer
        //zprintf(PSTR("Finish:\n"));
        m->status = 0;
        m = 0;
        r = startmove();
      }
    }
  }
#ifndef ISPC

    #ifdef IR_OLED_MENU
    //extern void IR_loop(int mode=0);
    //if (inwaitbuffer)IR_loop(1);
    #endif

  // this both check take 8us
  temp_loop(cm);
#ifdef motortimeout
  if (!m   && (cm - nextmotoroff >= motortimeout)) {
    nextmotoroff = cm;
    power_off();
  }
#endif // motortimeout

#if defined(ESP32) || defined(ESP8266)
  feedthedog();
#endif


#endif // ispc

  if (!wait_for_temp && !ishoming) {
    if (sendwait > 0) {
      sendwait -= delaywait;
      if (sendwait <= 0) {
        zprintf(PSTR("wait\n"));
        delaywait = 1;
        prevacc = 0;

        RUNNING = 1;
      }
    }
  }
}

/*
  =================================================================================================================================================
  STARTMOVE
  =================================================================================================================================================
  Mulai menjalankan 1 unit di buffer gerakan terakhir
*/
int subp = 1, laxis;

//float ts;
float xc[NUMAXIS];
int32_t estep;

// this part must calculated in parts to prevent single long delay
//=================================================================================================================
void calculate_delta_steps()
{
#ifdef NONLINEAR

#ifdef ISPC
  //zprintf(PSTR("\nSEG PRECALC %d\n"), s);
#endif


  // STEP 1
  xtotalseg --;
  memcpy(xc, m->dtx, sizeof xc);
  memcpy(x1, x2, sizeof x1);

  // STEP 2
  if (xtotalseg) {
#define PSEGMENT(i) xc[i] -= xtotalseg * sgx[i];
    PSEGMENT(0)
    PSEGMENT(1)
    PSEGMENT(2)
    PSEGMENT(3)
  }
  // STEP 3
  transformdelta(xc[0], xc[1], xc[2], xc[3]);
  // STEP 4
  // ready for movement
  totalstep = 0;
  for (int i = 0; i < 4; i++) {
    int32_t d = x2[i] - x1[i];
    if (d < 0) {
      bsdx[i] = -d;
      sx[i] = -1;
    } else {
      bsdx[i] = d;
      sx[i] = 1;
    }
    totalstep = fmax(bsdx[i], totalstep);
  }
  // modify the sx?? m->mcx and totalstep for this segment
  // STEP 5 FINAL
  mcx[0] = mcx[1] = mcx[2] = mcx[3] = 0;
  lctx=0;
  
  // convert from virtual step/mm value to original step/mm value
  stepdiv2 = wstepdiv2 * rampv;



#ifdef output_enable
  //zprintf(PSTR("\nX:%d %d %d \n"), fi(bsdx[0]), fi(bsdx[1]), fi(bsdx[2]));
  //zprintf(PSTR("Segm:%d Step:%d rampv:%f\n"), fi(xtotalseg), fi(totalstep), ff(rampv));
#endif
#else // nonlinear
  for (int i = 0; i < 4; i++) {


    if (m->dx[i] > 0) {
      bsdx[i] = (m->dx[i]);
      sx[i] = 1;
    } else {
      bsdx[i] = -(m->dx[i]);
      sx[i] = -1;
    }
  }
#endif // nonlinear


  // lets do the subpixel thing.

  // calculate the microseconds betwen step,
#ifdef SUBPIXELMAX
  xInvSqrt(dln, (m->fn ));
  int u;
  if (vSUBPIXELMAX > 1) {
    for (u = 2; u <= (vSUBPIXELMAX); u++) {
      if (LOWESTDELAY * u > dln) {
        break;
      }
    }
    u--;
  } else u = 1;
  subp = u;
  //zprintf(PSTR("Subpixel %d %d ->"),fi(u),fi(dln));
  if (u > 1) {
    totalstep *= u;
    stepdiv2 /= u;
    stepdiv /= u;
    stepdivL /=u;
    dln /= u;
    //if (m->laserval < 255)m->laserval = m->laserval / u + 1;
#ifdef NONLINEAR
    rampv /= u;
#endif

  }
  //zprintf(PSTR(" %d\n"),fi(dln));
#endif
  //
  mctr = totalstep ;
  newdircommand(!isG0 ? m->laserval : 0);
  //zprintf(PSTR("int %d\n"), fi(m->laserval));
#ifdef ISPC
  fctr = 0;
  m->col++;
#endif
}

int empty1 = 0;
int32_t startmove()
{
  if (!RUNNING)return 0;
  if (cmdfull)return 0;

  if ((head == tail)) { // if empty buffer then wait a little bit and send "wait"
    //Serial.println("?");
    if (!empty1) {
      zprintf(PSTR("empty\n"));
      empty1 = 1;
    }
    //m = 0; wm = 0; mctr = 0; // thi cause bug on homing delta
    if (!cmdempty) {
      delaywait = 20;
    } else {
      if (!sendwait) {
        LASER(!LASERON);

        // send one time, is buffer is emspy after running
#ifdef __ARM__
        sendwait = 100000;
#else
        sendwait = 1000000;
#endif
      }
    }
    return 0;
  } else empty1 = 0;
  sendwait = 0;
  // last calculation
  if (m ) return 0; // if empty then exit
#ifdef output_enable
  zprintf(PSTR("\nSTARTMOVE\n"));
#endif
  // STEP 1
  int t = nextbuff(tail);
  // prepare ramp (for last move)
  m = &moves[t];
  //zprintf(PSTR("FS %f\n"),ff(m->fs));
  laxis = fastaxis;
  fastaxis = FASTAXIS(m);
  totalstep = labs(m->dx[fastaxis]);

  isG0 = m->status & 8;
  prepareramp(t);




#ifdef NONLINEAR
  // DELTA HERE
  //STEP
  // if homing or just z move then no segmentation
  if (SINGLESEGMENT) xtotalseg = 1;
  else {
    xtotalseg = 1 + (( STEPSEGMENT ) / STEPPERSEGMENT);
  }

  // if from halt, then need to transform first point, if not then next first point is previous point
  if (m->status & 128) {
    transformdelta(otx[0], otx[1], otx[2], otx[3]);
  }

  // STEP

  ts = 1.f / (xtotalseg);
  rampseg = (float)(totalstep) * ts;
  ts *= IFIXED2;

#define CSGX(i) sgx[i] = float(m->dx[i]) * ts;
  CSGX(0)
  CSGX(1)
  CSGX(2)
  CSGX(3)

#ifdef output_enable
  zprintf(PSTR("\nTotalSeg:%d RampSeg:%f Sx:%f Sy:%f Sz:%f\n"), fi(xtotalseg), ff(rampseg), ff(sgx[0]), ff(sgx[1]), ff(sgx[2]));
  zprintf(PSTR("Dx:%d Dy:%d Dz:%d\n"), fi(m->dtx[0]), fi(m->dtx[1]), fi(m->dtx[2]));
#endif
#endif

  // STEP
  //ta = m->fs ;
  // this calculation cause
  //stepdiv = (CLOCKCONSTANT / (Cstepmmx(fastaxis)) );
  //stepdiv = (CLOCKCONSTANT / (Cstepmmx(fastaxis)) ) * Cstepmmx(fastaxis)*dis/(totalstep);

  stepdiv = CLOCKCONSTANT * m->dis / totalstep;
  stepdiv2 = wstepdiv2 = stepdiv;
  stepdivL = (stepdiv2) * Lscale * 0.00004; // this value tuned manually to get my 60W laser work from 5% to99%, but its better 25% for engraving
  lcdx=totalstep*0.00393*m->laserval;
  //stepdivL = 5 * Lscale; // this value tuned manually to get my 60W laser work from 5% to99%, but its better 25% for engraving

  m->status &= ~3;
  m->status |= 2;

#ifndef USETIMER1
  if (f == 0)
    nextmicros = micros();// if from stop
#endif

#ifdef output_enable
  //zprintf(PSTR("SUB:%d FS:%f TA:%f FE:%f RAMP:%d %d ACC:%f %f\n"), fi(subp), ff(m->fs), ff(m->fn), ff(fe), fi(rampup), fi(rampdown), ff(acup), ff(acdn));
#endif
  //  rampup = m->rampup  ;
  mctr2 = mcx[0] = mcx[1] = mcx[2] = mcx[3] = 0; //mctr >> 1;
  tail = t;

#ifdef output_enable
  /*
    zprintf(PSTR("Start tail:%d head:%d\n"), fi(tail), fi(head));
    zprintf(PSTR("RU:%d Rd:%d Ts:%d Dis:%f\n"), fi(rampup), fi(rampdown), fi(totalstep), ff(m->dis));
    zprintf(PSTR("FS:%f FN:%f AC:%f \n"), ff(m->fs), ff(m->fn), ff(m->ac));
    zprintf(PSTR("TA,ACUP,ACDN:%d,%d,%d \n"), fi(ta), fi(rampup), fi(rampdown));
    zprintf(PSTR("DX:%d DY:%d DZ:%d DE:%d \n"), fi(m->dx[0]), fi(m->dx[1]), fi(m->dx[2]), fi(m->dx[3]));

    //zprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
    //zprintf(PSTR("sx %d %d %d \n"), fi(sx[0]), fi(sx[1]), fi(sx[2]));
    //zprintf(PSTR("Status:%d \n"), fi(m->status));
  */
#endif
  calculate_delta_steps();
#ifdef SUBPIXELMAX
  rampup *= subp;
  rampdown *= subp;
#endif


  return 1;
}

/*
  =================================================================================================================================================
  DOCHECKENDSTOP
  =================================================================================================================================================
*/

void THEISR docheckendstop(int m)
{

  // AVR specific code here
  // read end stop



#ifndef ISPC
  m = 1;
#ifdef limit_pin
   if (m == 1) {
      int nc = 0;
      endstopstatus = 0;
      for (int d = 0; d < 3; d++) {
        if (ENDCHECK dread(limit_pin))nc++;
      }
      if (nc > 2)endstopstatus = -1;
    } else {
      endstopstatus = home_cnt > 0;
      home_cnt=0;
    }

#endif

#else
  // PC
#endif
}

/*
  =================================================================================================================================================
  HOMING
  =================================================================================================================================================
*/

String hstatus="";
void homing()
{

  pause_pwm(true);
  IR_end();
  init_home_input();
  // clear buffer
  waitbufferempty();
  ocz1=0;
  cz1=0;
  ishoming = 1;
  addmove(100, fmin(cx1,3), fmin(cy1,3), ax_home[2] <= 1 ? 0 : ocz1, ce01);
  waitbufferempty();
  ishoming = 1;
  cx1 = 0;
  cy1 = 0;
  ocz1 = 0;
  ce01 = 0;
  endstopstatus=0;
  x2[0] = x2[1] = x2[2] = x2[3] = 0;

  int32_t stx[NUMAXIS];
  //  stx[0] =  stx[1] = stx[2] = stx[3] = 0;
  int32_t tx[NUMAXIS];
  //  tx[0] =  tx[1] = tx[2] = tx[3] = 0;
#define mmax 1000
//HOMING_MOVE
#define smmax 2
//ENDSTOP_MOVE
  int vx[4] = { -1, -1, -1, -1};
  for (int t = 0; t < NUMAXIS; t++) {
    if (ax_home[t] > 1)vx[t] = 1;
    if (ax_home[t] == 0)vx[t] = 0;
    tx[t] = mmax * vx[t];
    stx[t] = smmax * vx[t];
  }

  
  // fast check every 31 step
  //zprintf(PSTR("Homing to %d %d %d\n"), tx[0], tx[1], tx[2]);

  // move away before endstop
#ifdef DRIVE_XYYZ
  ishoming = 0;
  checkendstop = 0;
  addmove(homingspeed, -stx[0], -stx[1], -stx[2], -stx[3], 1, 1);
  waitbufferempty();

  checkendstop = 31;
  addmove(homingspeed, 0, tx[1], tx[2], 0);
  waitbufferempty();
  checkendstop = 0;
  addmove(homingspeed, 0, -stx[1], -stx[2], 0, 1, 1);
  waitbufferempty();
  checkendstop = 31;
  ishoming = 1;
  addmove(homingspeed, tx[0], 0, 0, tx[3]);
#else
  checkendstop = 31;
  addmove(homingspeed, tx[0], tx[1], tx[2], tx[3]);
  hstatus=String(tx[0]);
#endif


  // now slow down and check endstop once again
  waitbufferempty();
  float xx[NUMAXIS];
#define moveaway(e,F) {\
    if (vx[e]) {\
      xx[0]=xx[1]=xx[2]=xx[3]=0;\
      xx[e] =  - stx[e];\
      checkendstop = 0;\
      addmove(F, xx[0], xx[1], xx[2], xx[3],1,1);\
      waitbufferempty();\
    }\
  }
#define xcheckendstop(e,F) {\
    xx[0] = xx[1] = xx[2] = 0; \
    xx[e] = tx[e]; \
    checkendstop = 1; \
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    waitbufferempty(); \
    xx[e] =  - stx[e] + axisofs[e]; \
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    waitbufferempty(); \
  }
  for (int e = 0; e < NUMAXIS; e++) {
    moveaway(e, homingspeed);
  }

  for (int e = 0; e < NUMAXIS; e++) {
    if (vx[e]) {
      // check endstop again fast
      xcheckendstop(e, 15);
      xcheckendstop(e, 3);
    }
  }
  checkendstop = ce01 = 0;


#ifdef NONLINEAR

  NONLINEARHOME

#else // NONLINEAR

  if (vx[0] > 0) cx1 = ax_home[0];
  else  cx1 = 0;
  if (vx[1] > 0) cy1 = ax_home[1];
  else  cy1 = 0;
  if (vx[2] > 0) ocz1 = ax_home[2];
  else  ocz1 = 0;
  cz1 = ocz1;

#endif // NONLINEAR

  //zprintf(PSTR("Home to:X:%f Y:%f Z:%f\n"),  ff(cx1), ff(cy1), ff(cz1));
  ishoming = 0;
  init_pos();
  
  
  pause_pwm(false);
  #ifdef spindle_pin
  xdigitalWrite(spindle_pin, HIGH);
  #endif
}


/*
  =================================================================================================================================================
  PROBING
  =================================================================================================================================================
*/
#ifdef MESHLEVEL
int XCount, YCount;
int ZValues[40][40]; //

float pointProbing()
{

  // clear buffer
  waitbufferempty();
  int32_t tx;
#define mmax HOMING_MOVE
#define smmax ENDSTOP_MOVE
  tx = -30;

  // fast check every 31 step
  //zprintf(PSTR("Homing to %d %d %d\n"), tx[0], tx[1], tx[2]);

  // move away before endstop
  int o = zcheckevery;

  checkendstop = 1;
  zcheckevery = 1;
  mm_ctr = 0;
  addmove(20, 0, 0, tx, 0, 1, 1);
  // now slow down and check endstop once again
  waitbufferempty();
  float zmm = mm_ctr;
  //zprintf(PSTR("Probe %f,%f = %f\n"), ff(cx1), ff(cy1), ff(zmm));
  zmm /= stepmmx[MZ];
  checkendstop = 0;
  //move back
  addmove(homingspeed, 0, 0, zmm, 0, 1, 1);


  zcheckevery = o;
  return zmm;
}

void meshprobe(float sx, float sy, float tx, float ty, int mc) {

}
#endif
/*
  =================================================================================================================================================
  Z bilinear interpotale for mesh leveling
  =================================================================================================================================================
*/

// need to extrapolate if outside area
#ifdef MESHLEVEL
// crude interpolizer resolution is mm for x and y and 0.1mm for z
float Interpolizer(int zX, int zY) {

  //Indexes
  int X0i = 1;
  int Y0i = 1;


  //Interpolated values

  //Check the boundary, and extrapolate if necessary

  if (zX > ZValues[XCount][0])zX = ZValues[XCount][0];
  if (zY > ZValues[0][YCount])zY = ZValues[0][YCount];
  //if (zX < ZValues[1][0])zX = ZValues[1][0];
  //if (zY < ZValues[0][1])zY = ZValues[0][1];
  //Load the table data into the variables
  for (int i = 2; i < XCount; i++) {

    if (zX >= ZValues[i][0] && zX <= ZValues[i + 1][0]) {
      X0i = i;
    }
  }

  for (int i = 2; i < YCount; i++) {

    if (zY >= ZValues[0][i] && zY <= ZValues[0][i + 1]) {
      Y0i = i;
    }
  }


  int X0 = ZValues[X0i][0];
  int X1 = ZValues[X0i + 1][0];
  int Y0 = ZValues[0][Y0i];
  int Y1 = ZValues[0][Y0i + 1];

  int X0Y0 = ZValues[X0i][Y0i];
  int X0Y1 = ZValues[X0i][Y0i + 1];
  int X1Y0 = ZValues[X0i + 1][Y0i];
  int X1Y1 = ZValues[X0i + 1][Y0i + 1];

  //Performs the calculations - no optimization to save space

  int XMY0 = X0Y0 + (zX - X0) * (X1Y0 - X0Y0) / (X1 - X0);
  int XMY1 = X0Y1 + (zX - X0) * (X1Y1 - X0Y1) / (X1 - X0);

  return (XMY0 + (zY - Y0) * (XMY1 - XMY0) / (Y1 - Y0)) * 0.1;

}
#else
float Interpolizer(int  zX, int zY) {
  return 0;
}

#endif

/*
  =================================================================================================================================================
  WAITBUFFEREMPTY
  =================================================================================================================================================
  waitbufferempty
  Proses semua gerakan di buffer, dan akhiri dengan deselerasi ke 0
*/
void waitbufferempty()
{
  //if (head != tail)prepareramp(head);
  startmove();
  //#define output_enable1
#ifdef output_enable1
  zprintf(PSTR("Wait %d\n"), fi(mctr));
#endif
  MEMORY_BARRIER()
  LOOP_IN(2)
  inwaitbuffer=true;
  while ((head != tail) || m || (cmhead != cmtail) || (endstopstatus < 0)) { //(tail == otail) //
    
    domotionloop
    MEMORY_BARRIER()
#ifdef output_enable1
    zprintf(PSTR("->H%d T%d cH%d cT%d\n"), fi(head), fi(tail), fi(cmhead), fi(cmtail));
#endif
    if (PAUSE)break;
    if (!RUNNING)break;
  }
  inwaitbuffer=false;
  LOOP_OUT(2)
#ifdef output_enable1
  zprintf(PSTR("Empty"));
#endif
}
/*
  =================================================================================================================================================
  NEEDBUFFER
  =================================================================================================================================================
  loop sampai da buffer yang bebas
*/
void needbuffer()
{
  if (nextbuff(head) == tail) {
#ifdef output_enable
    zprintf(PSTR("Wait %d / %d \n"), fi(tail), fi(head));
#endif
    //wait current move finish
    int t = tail;
    LOOP_IN(3)
    MEMORY_BARRIER()
    inwaitbuffer=true;
    while (t == tail) {
      domotionloop

      MEMORY_BARRIER()
      //zprintf(PSTR("%d\n"), fi(mctr));
    }
    //zprintf(PSTR("Done\n"));
    inwaitbuffer=false;
    LOOP_OUT(1)

#ifdef output_enable
    zprintf(PSTR("Done %d / %d \n"), fi(tail), fi(head));
#endif
  }

}

void faildetected()
{
#ifdef POWERFAILURE
  // save the print last line only if printing using SDCARD
  if (sdcardok == 2) {
    motor_0_OFF();
    motor_1_OFF();
    motor_2_OFF();
    motor_3_OFF();
    setfan_val(0);
    set_temp(0);
    // save last gcode in eeprom
    eepromwrite(EE_lastline, fi(lineprocess));
    delay(1000);
    checkendstop = 31;
    //void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0, int rel)
    addmove(200, -2000, 0, 0, 0, 1, 1);
    waitbufferempty();
  }
#endif
}
/*
  =================================================================================================================================================
  initmotion
  =================================================================================================================================================
  inisialisasi awal, wajib
*/
void init_pos()
{
  babystep[0] = babystep[1] = babystep[2] = babystep[3] = 0;
  e_ctr = ce01;
  prevacc = 0;
  nextacc = 0;
  curracc = 0;
  lastf = MINCORNERSPEED;
  head = 0;
  tail = 0;
  tailok = 0;
  cmhead = 0;
  cmtail = 0;
  info_e=ce01;
  info_x=cx1;
  info_y=cy1;
  info_z=cz1;
  info_x_s=info_x*stepmmx[0];
  info_y_s=info_y*stepmmx[1];
  info_z_s=info_z*stepmmx[2];
  set_pwm(0);
}
void initmotion()
{
#ifdef __ARM__
#ifndef _VARIANT_ARDUINO_STM32_
  disableDebugPorts();
#endif
#endif
  reset_motion();
  preparecalc();


#ifdef ISPC
  tickscale = 60;
  fscale = 2;
  graphscale = 4;
#endif
  motor_0_INIT();
  motor_1_INIT();
  motor_2_INIT();
  motor_3_INIT();
  motor_0_INIT2();
  motor_1_INIT2();
  motor_2_INIT2();
  motor_3_INIT2();

#ifndef ISPC
#ifdef POWERFAILURE
  pinMode(powerpin, INPUT_PULLUP);
  attachInterrupt(powerpin, faildetected, CHANGE);
#endif

  pinMotorInit
#endif
  #ifdef laser_pin
  xpinMode(laser_pin, OUTPUT);
  #endif
  SPINDLE(!SPINDLEON);
  LASER(!LASERON);

}
