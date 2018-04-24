
#include "motion.h"
#include "gcode.h"
#include "common.h"
#include "timer.h"
#include "config_pins.h"
#include "temp.h"
#include <stdint.h>


#ifndef ISPC
#include<arduino.h>
#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#endif


// bikin pusing !, buat check TIMER di AVR, malah freeze
//#define DEBUGLOOP

#ifdef DEBUGLOOP
#define LOOP_IN(n) zprintf(PSTR("LOOP %d IN "),fi(n));
#define LOOP_OUT(n) zprintf(PSTR("LOOP %d OUT "),fi(n));
#else
#define LOOP_IN(n)
#define LOOP_OUT(n)
#endif





// JERK Setting
//#define AJERK 5
#define MINCORNERSPEED 2 // minimum cornering speed
#define MINSTEP 0
// Centripetal
//#define JERK1 //centripetal corner , still not good, back to repetier style jerk
#define DEVIATE 5//;0.02*50 - for centripetal corner safe speed

// repetier 1
#define JERK2 //repetier style jerk
//#define JERK2A //repetier style jerk



#ifdef USETIMER1
#define CLOCKCONSTANT 8000000.f        // microseconds
#ifdef __AVR__
#define DSCALE 2   // use 2Mhz timer need to shift right 2bit
#define DIRDELAY 20
#endif

#ifdef __ARM__
#define DSCALE 0   // use 8Mhz timer shift 0bit
#define DIRDELAY 20 // usec
#endif

#else // usetimer1
#define CLOCKCONSTANT 1000000.f        // microseconds

#define DSCALE 0  // 1mhz use micros shift 3bit
#define DIRDELAY 2
#endif

#define X 0
#define Y 1
#define Z 2
#define E 3


uint8_t homingspeed;
uint8_t xback[4];
uint8_t head, tail, tailok;
int maxf[4];
float xyscale, f_multiplier, e_multiplier;
int xyjerk, accel;
int i;
int mvaccel;
float ax_home[NUMAXIS];
float stepdiv, stepdiv2;
float wstepdiv2;
int32_t totalstep;
uint32_t bsdx[NUMAXIS];
int8_t  sx[NUMAXIS];
int32_t dlp, dl, dln;
int8_t checkendstop, xctstep, yctstep, zctstep, ectstep, xcheckevery, ycheckevery, zcheckevery, echeckevery;
int16_t endstopstatus;
int8_t ishoming;
float axisofs[4] = {0, 0, 0, 0};
float F_SCALE = 1;
int8_t RUNNING=1;
float stepmmx[4];
float cx1, cy1, cz1, ce01;
tmove move[NUMBUFFER];

#define sqr2(n) (n)*(n)


#include "nonlinear.h"

// to calculate delay from v^2, fast invsqrt hack
#define INVSCALE 3
#define INVSCALE2 (1<<(INVSCALE/2))
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
#else
// other CPU use normal SQRT
#define InvSqrt(f) 1.0/sqrt(f)
#endif

#define INVSCALE3 1 << INVSCALE
#define xInvSqrt(n) n>1?stepdiv2*InvSqrt(n):stepdiv

#define sqrt32(n) sqrt(n)

/*
    =================================================================================================================================================
    RESET_MOTION
    =================================================================================================================================================
  Reset all variable
*/
// keep last direction to enable backlash if needed

void reset_motion()
{

  // 650bytes code
  homingspeed = HOMINGSPEED;
  xyjerk = XYJERK;
  xyscale = 1;
  ishoming = 0;
  cmctr = 0;
  e_multiplier = f_multiplier = 1;
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
  ce01 = 0;
#ifdef ISPC
  tick = 0;
#endif
#ifndef SAVE_RESETMOTION

#ifdef NONLINEAR
  delta_diagonal_rod = DELTA_DIAGONAL_ROD;
  delta_radius = DELTA_RADIUS;
#endif


  maxf[0] = XMAXFEEDRATE;
  maxf[1] = YMAXFEEDRATE;
  maxf[2] = ZMAXFEEDRATE;
  maxf[3] = E0MAXFEEDRATE;



  accel = XACCELL;

  mvaccel = XMOVEACCELL;

  stepmmx[0] = XSTEPPERMM;
  stepmmx[1] = YSTEPPERMM;
  stepmmx[2] = ZSTEPPERMM;
  stepmmx[3] = E0STEPPERMM;

  ax_home[0] = XMAX;
  ax_home[1] = YMAX;
  ax_home[2] = ZMAX;

  xback[0] = MOTOR_X_BACKLASH;
  xback[1] = MOTOR_Y_BACKLASH;
  xback[2] = MOTOR_Z_BACKLASH;
  xback[3] = MOTOR_E_BACKLASH;

#endif
}

void preparecalc()
{
#ifdef NONLINEAR

  nonlinearprepare();

#endif

}

int32_t mcx[NUMAXIS];
/*
    =================================================================================================================================================
    MOTOR CLASS
    =================================================================================================================================================
*/



#define FASTAXIS(n) ((n->status >> 4)&3)
//if(m)coreloop();
#include "motors.h"

int mb_ctr;
int32_t bufflen()
{
  mb_ctr = head >= tail ? head - tail : (NUMBUFFER + head) - tail; // 5+0 - 4
  return mb_ctr;
}

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
*/
int32_t currdis, prevdis;
#define ramplen(oo,v0,v1,a,stepmm) oo=((int32_t)v1-(int32_t)v0)*stepmm/(a);
#define ramplenq(oo,v0,v1,stepa) oo=((int32_t)v1-(int32_t)v0)*stepa;
#define speedat(v0,a,s,stp) ((int32_t)a * s / stp + (int32_t)v0)
#define accelat(v0,v1,s) ((int32_t)v1  - (int32_t)v0 ) / (2 * s)
#define sqr(x) x*x
void prepareramp(int32_t bpos)
{
  tmove *m;
  m = &move[bpos];
  if (m->status & 4)return; // already calculated

  int faxis = FASTAXIS(m);
  int32_t ytotalstep = labs(m->dx[faxis]);
#define stepmm  Cstepmmx(faxis)

  float stepa = stepmm / (m->ac);
  CORELOOP

  ramplenq(m->rampup, m->fs, m->fn, stepa);
  ramplenq(m->rampdown, m->fe, m->fn, stepa);

  CORELOOP

#ifdef output_enable
#ifdef ISPC
#define preprampdebug
#endif
  //  #define preprampdebug
#endif
#ifdef preprampdebug
  zprintf(PSTR("PREPARE RAMP FX %d Stepmm %f Bpos:%d\n"), fi(faxis), ff(stepmm), fi(bpos));
  zprintf(PSTR("Bpos:%d\n"), (int32_t)bpos);
  zprintf(PSTR("----------1-----------\nRU:%d Rd:%d TS:%d\n "), m->rampup, m->rampdown, ytotalstep);
  zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
  // New generic algorithm, to calculate rampup and rampdown simpler, and faster
  // if rampup and ramp down crossing
  if (m->rampup + m->rampdown > ytotalstep) {
    if (m->rampup) {
      // if crossing and have rampup
      int32_t r = ((m->rampdown + m->rampup) - ytotalstep) >> 1;
      m->rampup -= r;
      //if (m->rampup<0)m->rampup=0;
      m->rampdown -= r;
    }
    // check if still larger that totalstep
    if (m->rampup > ytotalstep) {
      // adjust speed if acceleration up cannot reach requested speed
      m->fn = speedat(m->fs, m->ac, ytotalstep, stepmm);
      CORELOOP
      m->fe = m->fn;
      m->rampdown = 0;
      m->rampup = ytotalstep;
      CORELOOP
#ifdef preprampdebug
      zprintf(PSTR("========2========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else if (m->rampdown > ytotalstep) {
      // ---------adjust deceleration
      // BACKPLANNER iterate back and propagate rampdown to previous move
#ifdef preprampdebug
      zprintf(PSTR("\nBACKPLANNNER\n"));
      zprintf(PSTR("RU:%d Rd:%d TS:%d\n "), fi(m->rampup), fi(m->rampdown), fi(ytotalstep));
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
#ifdef BACKPLANNER
      m->rampup = 0;
      m->rampdown = ytotalstep;
      m->fn = m->fs = speedat(m->fe, m->ac * BACKPLANNERRATIO, ytotalstep, stepmm);
      CORELOOP
      int32_t ls = m->fs;
      tmove* mi;
      bpos = prevbuff(bpos);
      MEMORY_BARRIER()
      while (bpos != tail) {
        mi = &move[bpos];
        int32_t ts = labs(mi->dx[FASTAXIS(mi)]);
#ifdef preprampdebug
        zprintf(PSTR("BPOS:%d %d\n"), fi(bpos), fi(ts));
#endif
        mi->fe = ls;
        float stepmmi = Cstepmmx(FASTAXIS(mi));
        ramplen(mi->rampdown, mi->fe, mi->fn, mi->ac * BACKPLANNERRATIO, stepmmi );
        MEMORY_BARRIER()
        CORELOOP
        if (mi->rampdown <= ts) {
          int32_t sub = ts - (mi->rampdown + mi->rampup);
          if (sub < 0) {
            mi->rampdown -= sub;
            mi->rampup -= sub; //fmax(0,m->rampup-sub);
          }
          mi->fn  = speedat(mi->fe, mi->ac * BACKPLANNERRATIO, mi->rampdown, stepmmi);
          break;
        } else {
          mi->rampup = 0;
          mi->rampdown = ts;
          mi->fn = mi->fs = speedat(mi->fe, mi->ac * BACKPLANNERRATIO, ts, stepmmi);
          MEMORY_BARRIER()
          CORELOOP
          ls  = mi->fs;
          bpos = prevbuff(bpos);
        }
      }
      CORELOOP
      if (bpos == tail) {
        //zprintf(PSTR("\n\nReach tail!\n\n"));
      }

#else
      // just set the actual exit speed
      m->rampup = 0;
      m->fn = m->fs;
      m->rampdown = ytotalstep;

#endif
    } else {
      // calculate new nominal speed if up and down crossing
      m->fn = speedat(m->fs, m->ac, m->rampup, stepmm);
    }
  }


#ifdef preprampdebug
  zprintf(PSTR("========FINAL========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
  zprintf(PSTR("FS:%f AC:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->ac), ff(m->fn),  ff(m->fe));
#endif
  //if (m->fs < 1) m->fs = 1;
  m->status |= 4;
  CORELOOP
}

/*
    =================================================================================================================================================
    PLANNER
    =================================================================================================================================================
  dipanggil oleh   untuk mulai menghitung dan merencakanan gerakan terbaik
  kontrol mbelok adalah cara mengontrol supaya saat menikung, kecepatan dikurangi sehingga tidak skip motornya

*/

float currf[5], prevf[5];
void planner(int32_t h)
{
  // mengubah semua cross feedrate biar optimal secepat mungkin
  int32_t p;
  tmove *curr;
  tmove *prev;

  curr = &move[h];
  float  scale = 1;
  int32_t xtotalstep = abs(curr->dx[FASTAXIS(curr)]);
  memcpy(prevf, currf, sizeof prevf);
#ifdef JERK2
  currf[4] = curr->fn;
#endif
  for (int i = 0; i < NUMAXIS; i++) {
    //prevf[i] = currf[i];
    currf[i] = 0;
    if (curr->dx[i] != 0) {

      float cdx = int32_t(curr->fn) * curr->dx[i];
      float scale2 = (maxf[i]) * xtotalstep / fabs(cdx);
      if (scale2 < scale) scale = scale2;
#ifdef JERK1
      currf[i] = curr->dx[i]; // * 100 / stepmmx[i];
#else
      currf[i] = float(cdx) / xtotalstep;
#endif
      CORELOOP
    }
  }
  // update all speed and square it up, after this all m->f are squared !
#ifdef output_enable
  zprintf (PSTR("Fratio :%f\n"), ff(scale / 16.0));
#endif
  scale *= curr->fn;
  curr->fn = (scale * scale);
#ifdef output_enable
  zprintf (PSTR("FN :%d\n"), fi(curr->fn));
#endif
  //#define JERK1

#ifdef JERK1
  prevdis = currdis;
  currdis = sqrt32(2 + sqr2(currf[0]) + sqr2(currf[1]) + sqr2(currf[2]));
  CORELOOP
#endif
  if (bufflen() < 1) return;
  p = prevbuff(h);
  prev = &move[p];
  if ((prev->status & 3) == 2)return;// if already planned

  // ==================================
  // Centripetal corner max speed calculation, copy from other firmware
  if (prev->status & 3) {

    int32_t max_f = MINCORNERSPEED * MINCORNERSPEED;
    /*

         ======================================================
      change the junction speed to centripetal acceleration model
      junc cos = (prev.dx*cur.dx/(prev.totalstep*cur.totalstep) - (prev.dy ...


    */
#ifdef JERK1
    int32_t divi = (prevdis * currdis);
#ifdef output_enable
    zprintf (PSTR("DX[0]%d Divi :%f\n"), fi(curr->dx[0]), ff(divi));
#endif
    CORELOOP
    if (divi > 0) {
      //#define dot(ix) (prev->dx[ix]*curr->dx[ix])
#define dot(ix) ((prevf[ix])*(currf[ix]))

      int32_t junc_cos =  (dot(0) + dot(1) + dot(2)) * 400 / divi;
      //junc_cos/=prevdis;
      CORELOOP
#ifdef output_enable
      zprintf (PSTR("Cos :%f\n"), ff(junc_cos));
#endif

      if (junc_cos < 350) {
        //junc_cos*=2;
        junc_cos = fmax(junc_cos, -390); // Check for numerical round-off to avoid divide by zero.
        //zprintf (PSTR("Cos :%f\n"), ff(junc_cos));
        int32_t sin_theta_d2 = sqrt32((400 - junc_cos) / 2); // result will be range 0-500// Trig half angle identity. Always positive.
        //CORELOOP
        //zprintf (PSTR("Sin :%f\n"), ff(sin_theta_d2));
        //zprintf (PSTR("Ac :%d\n"), fi(prev->ac));

        // TODO: Technically, the acceleration used in calculation needs to be limited by the minimum of the
        // two junctions. However, this shouldn't be a significant problem except in extreme circumstances.
        max_f = fmax(MINCORNERSPEED * MINCORNERSPEED, ((int32_t)curr->ac ) * sin_theta_d2 * DEVIATE   / ((20 -  sin_theta_d2) << 6));
        CORELOOP

        //zprintf ("Cos :%f\n", ff(junc_cos ));
      } else max_f = 10000;
    };
    //if (max_f < 0)max_f = 9;
#ifdef output_enable
    zprintf (PSTR("MF:%d\n"), fi(max_f));
#endif
#elif defined(JERK2)
    max_f = fmax(currf[4], prevf[4]);
#ifdef JERK2A
    float jerk = max_f * 0.7 * (1 - (currf[0] * prevf[0] + currf[1] * prevf[1] + currf[2] * prevf[2]) / ((currf[4] * prevf[4])));
#else
    int32_t fdx = currf[0] - prevf[0];
    int32_t fdy = currf[1] - prevf[1];
    int32_t fdz = currf[2] - prevf[2];

    float jerk = sqrt(fdx * fdx + fdy * fdy + fdz * fdz);
#endif
    CORELOOP
    float factor = 1;
    if (jerk > xyjerk) {
      factor = float(xyjerk) / jerk; // always < 1.0!
      CORELOOP
      if (factor * max_f * 2.0 < xyjerk)
      {
        factor = xyjerk / (2.0 * max_f);
        CORELOOP
      }
    }

    //if (jerk > XYJERK) max_f = XYJERK * max_f / jerk;
    //jerk = abs(fdz) ;
    //if (jerk > ZJERK) max_f = ZJERK * max_f / jerk;

    max_f = fmax(max_f * factor, MINCORNERSPEED);
    CORELOOP
#ifdef output_enable
    zprintf (PSTR("JERK:%d FACTOR:%f\n"), fi(jerk), ff(factor));
    zprintf (PSTR("XMF:%d\n"), fi(max_f));
#endif
    max_f *= max_f;

#else // jerk2
    float ratio = 1;
    for (int i = 0; i < 3; i++) {
      //if (currf[i]*prevf[i] < 0)
      {
        float jerk = abs(currf[i] - prevf[i]);
        if (jerk > xyjerk) ratio = fmin(ratio, float(xyjerk) / jerk);
        //else if (jerk > XYJERK) max_f = fmin(max_f, prevf[0] - XYJERK);
        CORELOOP
      }
    }
    max_f = fmax((max_f * ratio), MINCORNERSPEED);
#ifdef output_enable
    zprintf (PSTR("XMF:%d\n"), fi(max_f));
#endif
    max_f *= max_f;
#endif

    prev->fe = fmin(max_f, prev->fn);
    prev->fe = fmin(curr->fn, prev->fe);
    //zprintf (PSTR("PF:%d\n"), fi(prev->fe));
    prepareramp(p);
    curr->fs = prev->fe;
    CORELOOP
  }

}

/*
    =================================================================================================================================================
    ADDMOVE
    =================================================================================================================================================
  Rutin menambahkan sebuah vektor ke dalam buffer gerakan
*/
int32_t x1[NUMAXIS], x2[NUMAXIS];
tmove *m;

#ifdef ISPC
float fctr, tick;
float tickscale, fscale, graphscale;
#endif

#ifdef NONLINEAR
float rampv;
float istepmmx[4];
float rampseg, rampup, rampdown;
#else
int32_t rampseg, rampup, rampdown;
#define rampv 1
#endif

int32_t  f;
int32_t ta, acup, acdn, oacup, oacdn;
int32_t mctr, xtotalseg;
uint8_t fastaxis;
uint32_t nextmicros;
uint32_t nextdly;
uint32_t nextmotoroff;
float otx[NUMAXIS]; // keep the original coordinate before transform
int8_t repos = 0;

void addmove(float cf, float cx2, float cy2, float cz2, float ce02, int g0, int rel)
{


  needbuffer();
  int32_t x2[NUMAXIS];
  XYSCALING
  if (head == tail) {
    //zprintf(PSTR("Empty !\n"));
  }
#ifdef output_enable
  xprintf(PSTR("ADDMOVE\nTail:%d Head:%d \n"), fi(tail), fi(head));
  xprintf(PSTR("F:%f From X:%f Y:%f Z:%f\n"), ff(cf), ff(cx1), ff(cy1), ff(cz1));
  xprintf(PSTR("To X:%f Y:%f Z:%f\n"),  ff(cx2), ff(cy2), ff(cz2));
#endif
  tmove *curr;
  curr = &move[nextbuff(head)];
  curr->status = g0 ? 8 : 0; // reset status:0 planstatus:0 g0:g0

  if (rel) {
    cx1 = cy1 = cz1 = ce01 = 0;
  }

#ifdef ISPC
  curr->col = 2 + (head % 4) * 4;
#endif
  // this x2 is local
  //curr->dis=sqrt(sqr2(cx2-cx1)+sqr2(cy2-cy1)+sqr2(cz2-cz1));
#ifndef NONLINEAR
  x2[0] = (int32_t)(cx2 * Cstepmmx(0)) - (int32_t)(cx1 * Cstepmmx(0))  ;
  x2[1] = (int32_t)(cy2 * Cstepmmx(1)) - (int32_t)(cy1 * Cstepmmx(1)) ;
  x2[2] = (int32_t)(cz2 * Cstepmmx(2)) - (int32_t)(cz1 * Cstepmmx(2)) ;
#else
  x2[0] = (cx2 - cx1) * Cstepmmx(0);
  x2[1] = (cy2 - cy1) * Cstepmmx(1);
  x2[2] = (cz2 - cz1) * Cstepmmx(2);
#endif
  CORELOOP
  x2[3] = (ce02 - ce01) * Cstepmmx(3)  * e_multiplier;
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
#elif defined(DRIVE_XYYZ)
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


  // if no axis movement then dont multiply by multiplier
  if (g0 || ishoming || ((x2[0] == 0) && (x2[1] == 0) && (x2[2] == 0)) ) {} else cf *= f_multiplier;
#ifdef __AVR__
  if (cf > 100)cf = 100; // prevent max speed
#endif

  CORELOOP
  curr->fn = cf; //curr->fn *= curr->fn;
  curr->fe = 0;
  curr->fs = 0;
  //curr->planstatus = 0; //0: not optimized 1:fixed
  //calculate delta
  int32_t dd;
  dd = 0;
  int faxis = 0;
  // calculate the delta, and direction
  for (int i = 0; i < NUMAXIS; i++) {
#define delta  x2[i]
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
    xprintf(PSTR("Totalstep AX%d %d\n"), (int32_t)faxis, (int32_t)curr->dx[faxis]);
#endif
    curr->status |= faxis << 4;
    curr->ac = 2 * (g0 ? mvaccel : accel);
    //zprintf(PSTR("F:%f A:%d\n"), ff(cf), fi(curr->ac));
    if (head == tail) {
      otx[0] = cx1;
      otx[1] = cy1;
      otx[2] = cz1;
      otx[3] = ce01;
      curr->status |= 128;
    }
    head = nextbuff(head);
    curr->status |= 1; // 0: finish 1:ready
    // planner are based on cartesian coord movement on the motor
    planner(head);
#ifdef NONLINEAR
    cx1 = curr->dtx[0] = cx2; // save the target, not the original
    cy1 = curr->dtx[1] = cy2;
    cz1 = curr->dtx[2] = cz2;
    ce01 = curr->dtx[3] = ce02;
#else
    cx1 = cx2;
    cy1 = cy2;
    cz1 = cz2;
    ce01 = ce02;
#endif
  }

}

#define N_ARC_CORRECTION 25
#ifdef ARC_SUPPORT

/*

    ARC using Float implementation

*/

uint32_t approx_distance(uint32_t dx, uint32_t dy) {
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
  uint32_t radius = approx_distance(labs(fI), labs(fJ));
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc

  float cx = cx1 + fI ;
  float cy = cy1 + fJ ;
#ifdef debug1
  zprintf(PSTR("Arc  I, J,R:%d,%d,%d\n"), fI, fJ, radius);
  zprintf(PSTR("go cX cY:%d %d\n"), cx, cy);
#endif

  //uint32_t linear_travel = 0; //target[axis_linear] - position[axis_linear];
  float ne = ce01;
  uint32_t extruder_travel = (ce02 - ce01) * Cstepmmx(3)  * e_multiplier;

  float r_axis0 = -fI;  // Radius vector from center to current location
  float r_axis1 = -fJ;
  float rt_axis0 = cx2 - cx;
  float rt_axis1 = cy2 - cy;

#ifdef debug1
  sersendf_P(PSTR("go rX rY:%lq %lq\n"), r_axis0 * 1000, r_axis1 * 1000);
  sersendf_P(PSTR("go rtX rtY:%lq %lq\n"), rt_axis0 * 1000, rt_axis1 * 1000);
#endif

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
  if ((!isclockwise && angular_travel <= 0.00001) || (isclockwise && angular_travel < -0.000001))
  {
    angular_travel += 2.0f * M_PI;
  }
  if (isclockwise)
  {
    angular_travel -= 2.0f * M_PI;
  }

  uint32_t millimeters_of_travel = fabs(angular_travel) * radius; //hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 1)
  {
    return;// treat as succes because there is nothing to do;
  }
  uint32_t segments = fmax(1, millimeters_of_travel >> 11);
  float theta_per_segment = angular_travel / segments;
  float extruder_per_segment = (extruder_travel) / segments;

  float cos_T = 2.0 - theta_per_segment * theta_per_segment;
  float sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
  cos_T *= 0.5;

  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  //arc_target[axis_linear] = position[axis_linear];

  // Initialize the extruder axis


  for (i = 1; i < segments; i++)
  {
    // Increment (segments-1)

    if (count < N_ARC_CORRECTION)  //25 pieces
    {
      // Apply vector rotation matrix
      r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
      r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
      r_axis1 = r_axisi;
      count++;
    }
    else
    {
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
    //move.axis[E] = extscaled>>4;
    addmove(cf, nx, ny, cz2, ne);
  }
  // Ensure last segment arrives at target location.
  addmove(cf, cx2, cy2, cz2, ce02);
}
#endif



#ifdef NONLINEAR
void calculate_delta_steps();
#endif


static int32_t cmdly;


#ifdef ISPC

float xs[4] = {0, 0, 0, 0};
float gx, gy, lx, ly;
int pcsx[4];
#define graphstep(ix) xs[ix] +=pcsx[ix]
void dographics()
{
  f = 10 * stepdiv / cmdly;
  //f = stepdiv / (float)dl;
  //f = 2 * sqrt(ta);
  tick = tick + cmdly; //1.0/timescale;
  int32_t c;
  // color by segment
  c = (tail & 3) + 10;
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
  gy = 480 -  fscale * f * 0.1;
  setcolor(c);
  putpixel (gx, gy, c);
  //line(lx, ly, gx, gy);
  //zprintf(PSTR("X:%f Y:%f\n"), ff(gx), ff(gy));
  setcolor(0);
  line(gx + 1, 480 - 1, gx + 1, 480 - 100);

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
  putpixel (ex + ey * 0.3 + 120, ey * 0.3 - ez + 150, c);
#endif

}
#else
#define graphstep(ix)
#endif
/*
    =================================================================================================================================================
    MOTIONLOOP
    =================================================================================================================================================
*/
void otherloop(int r);
uint32_t cm, ocm,  mctr2, dlmin, dlmax;
int32_t timing = 0;


/*
    =================================================================================================================================================
    COMMAND BUFFER
    =================================================================================================================================================
*/
#define NUMCMDBUF 25
#define nextbuffm(x) ((x) < NUMCMDBUF-1 ? (x) + 1 : 0)

static volatile uint32_t cmddelay[NUMCMDBUF], cmd0;
static volatile uint8_t cmhead = 0, cmtail = 0, cmcmd, cmbit;

static int8_t mo = 0;
#define cmdfull (nextbuffm(cmhead)==cmtail)
#define cmdnotfull (nextbuffm(cmhead)!=cmtail)
#define cmdempty (cmhead==cmtail)
static int nextok = 0;


static void decodecmd()
{
  if (cmdempty) {
    return;
  }
  uint32_t cmd = cmddelay[cmtail];

  cmcmd = cmd & 1;
  cmbit = (cmd >> 1) & 15;
  // inform if non move is in the buffer
  //if (cmcmd && (cmbit==0))zprintf(PSTR("X"));
  cmdly = (cmd >> 5) >> DSCALE;
  nextok = 1;
  cmtail = nextbuffm(cmtail);
#ifdef USETIMER1
  timer_set(cmdly);
#endif
#ifdef output_enable //ISPC
  //zprintf(PSTR("DLY:%d \n"), fi(cmdly));
#endif
}

uint32_t mc, dmc, cmctr;
void coreloopm()  // m = micros - nextmicros  value
{
  //dmc=(micros()-mc); mc=micros();
  if (!nextok) {
    decodecmd();
    if (!nextok) return;
  }
#ifdef USETIMER1
  {
#elif defined(ISPC)
  {
#else
  if (!RUNNING){
    
    // halt and clear buffer
    checkendstop=1;
    endstopstatus=-1;
    return;
  }
  cm = micros();
  
  if (cm - nextmicros >= cmdly) {
    nextmicros = cm;
#endif

    if (cmcmd) { // 1: set dir
      if (cmbit & 8) {
        ectstep++;
        motor_3_STEP();
      }
      if (cmbit & 1) {
        cmctr++;
        xctstep++;
        motor_0_STEP();
      }
      if (cmbit & 2) {
        yctstep++;
        motor_1_STEP();
      }
      if (cmbit & 4) {
        zctstep++;
        motor_2_STEP();
      }
      pinCommit();
      //somedelay(1) ;
      motor_0_UNSTEP();
      motor_1_UNSTEP();
      motor_2_UNSTEP();
      motor_3_UNSTEP();
      pinCommit();
      //STEPDELAY ;

#ifdef ISPC
      if (cmbit & 1)graphstep(0);
      if (cmbit & 2)graphstep(1);
      if (cmbit & 4)graphstep(2);
      if (cmbit & 8)graphstep(3);
      dographics();
#endif

      if (checkendstop) { // check endstop every 30 step
        if ((xctstep >= xcheckevery) || (yctstep >= ycheckevery) || (zctstep >= zcheckevery) || (ectstep >= echeckevery) )
        {
          xctstep = yctstep = zctstep = ectstep = 0;
          docheckendstop();
          if (endstopstatus < 0) {
            cmtail = cmhead;
            nextok = 0;
            return; // no more move and clear cmdbuffer if endstop hit/
          }
        }
      }

    } else { // 0: motor step
      motor_0_DIR((cmbit & 1) ? -1 : 1);
      motor_1_DIR((cmbit & 2) ? -1 : 1);
      motor_2_DIR((cmbit & 4) ? -1 : 1);
      motor_3_DIR((cmbit & 8) ? -1 : 1);
      pinCommit();
      dobacklash();
#ifdef ISPC
      pcsx[0] = (cmbit & 1) ? 1 : -1;
      pcsx[1] = (cmbit & 2) ? 1 : -1;
      pcsx[2] = (cmbit & 4) ? 1 : -1;
      pcsx[3] = (cmbit & 8) ? 1 : -1;
#endif
    }
    // next command
    nextok = 0;
    decodecmd();
  }
}

//
/* idea:
   merge a non move
*/

int ldelay = 0;
static void pushcmd()
{
  // wait until a buffer freed

  while (cmdfull) {
#ifdef USETIMER1
    MEMORY_BARRIER();
#endif
    CORELOOP
  }

  // if move cmd, and no motor move, save the delay
  if ( (cmd0 & 1) && !(cmd0 & (15 << 1))) {
    ldelay += cmd0 >> 5;
  } else {
    cmhead = nextbuffm(cmhead);
    cmddelay[cmhead] = cmd0 + (ldelay << 5);
    ldelay = 0;
  }
}

void newdircommand()
{
  // change dir command
  cmd0 = DIRDELAY << 5;
  if (sx[0] > 0)cmd0 |= 2;
  if (sx[1] > 0)cmd0 |= 4;
  if (sx[2] > 0)cmd0 |= 8;
  if (sx[3] > 0)cmd0 |= 16;
  pushcmd();

}
/* ================================================================================================================
                                                BRESENHAM CORE

                              this need to move to interrupt for smooth movement
   ================================================================================================================
*/


#define bresenham(ix)\
  if ((mcx[ix] -= bsdx[ix]) < 0) {\
    cmd0 |=2<<ix;\
    mcx[ix] += totalstep;\
  }


#ifdef INTERPOLATEDELAY
#define CALCDELAY dl = ((dl<<3)+(dlp-dl))>>3; // hack, linear interpolation of the delay
//#define CALCDELAY dl = (dlp+dl)>>1; // hack, linear interpolation of the delay
#else
#define CALCDELAY dl = dlp;
#endif
// ===============================
int32_t _ac = 0;
int coreloop1()
{

  if (!m || (mctr <= 0)) {
    return 0;
  }


  if (cmdfull) {
    //zprintf(PSTR("F\n"));

  } else   {
    cmd0 = 1; //step command

    bresenham(0);
    bresenham(1);
    bresenham(2);
    bresenham(3);

    // next speed
#ifdef AJERK
    if ((rampup -= rampv) > 0) {
      if (_ac < acup)_ac += AJERK;
    } else if ((rampdown -= rampv) < 0) {
      if (_ac > acdn)_ac -= AJERK;
    } else {
      if (_ac < 0) _ac += AJERK;
      else if (_ac > 0) _ac -= AJERK;
    }
#else
    if ((rampup -= rampv) > 0) {
      _ac = acup;
    } else if ((rampdown -= rampv) < 0) {
      _ac = acdn;
    } else {
      _ac = 0;
    }
#endif
    //    // calculate new delay only if we accelerating
UPDATEDELAY:
    if (_ac) {
      ta += _ac;
      //zprintf(PSTR("%d\n"),fi(ta));
      // if G0 update delay not every step to make the motor move faster
      //if (m->status & 8) {
#ifdef UPDATE_F_EVERY
      nextdly += dl;
      if (nextdly > UPDATE_F_EVERY) {
        nextdly -= UPDATE_F_EVERY;
        dlp = xInvSqrt(ta); //*F_SCALE;
      };
#else
      dlp = xInvSqrt(ta); //*F_SCALE;
#endif
    }  else dlp = dln;

    CALCDELAY
    cmd0 |= (dl) << 5;
    pushcmd();
    mctr--;
    CORELOOP
    //if (mctr  == 0)zprintf(PSTR("\n"));
#ifdef output_enable
    //zprintf(PSTR("Rampv:%f \n"), ff(rampv));
    //zprintf(PSTR("%d "), dl);
    //zprintf(PSTR("MCTR:%d \n"), mctr);
    // write real speed too
    dlmin = fmin(dl, dlmin);
    dlmax = fmax(dl, dlmax);
    if (mctr2++ % 29 == 0) {
      //zprintf(PSTR("%f "), ff(sqrt(ta)));
      //zprintf(PSTR("%d "), dl);
      //zprintf(PSTR("RU:%f RD:%f \n"), ff(rampup), ff(rampdown));
      //printf(PSTR("D:%d RD:%f "), ff(rampup), ff(rampdown));
      //zprintf(PSTR("dmin:%d dmax:%d "), fi(dlmin), fi(dlmax));
      //zprintf(PSTR("Fmin:%d Fmax:%d\n"), fi(stepdiv / dlmax), fi(stepdiv / dlmin));
      //zprintf(PSTR("F:%d\n"), fi(stepdiv/dlmin));

      dlmin = 1000000;
      dlmax = 0;
    }
#endif
#ifndef ISPC
    //if (mctr % 10 == 0)zprintf(PSTR("%d "), fi(mctr));
    //if (mctr % 4 == 0) zprintf(PSTR("F:%d "), fi(stepdiv/dl));
    //if (mctr2++ % 20 == 0) Serial.println("");
#endif
    //    if (mctr % 10 == 0)zprintf(PSTR("endstp %d\n"),fi(checkendstop));
    if (checkendstop) {
      //      docheckendstop();
      //zprintf(PSTR("%d \n"),fi(mctr));
      if (endstopstatus < 0) {
        //zprintf(PSTR("Endstop hit"));
        endstopstatus = 0;
        m->status = 0;
        mctr = 0;
        m = 0;
        head = tail;
        RUNNING=1;
        return 0;
      }
    }
    return 1;
  }
  return 0;
}

int busy = 0;
// ===============================================
int motionloop()
{

  if (busy ) {
    zprintf(PSTR("Busy\n"));
    return 0;
  }
  busy = 1;
  CORELOOP
  int  r;
  if (!m ) r = startmove();
  else {
    r = coreloop1();
    //if (mctr > 0)
  }
  // for auto start motion when idle
  otherloop(r);
  busy = 0;
  return r;
}

void otherloop(int r)
{
  cm = micros();
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
        //zprintf(PSTR("Finish:%d %f\n"), fi(mctr2), ff(fctr));
        m->status = 0;
        if (m->fe == 0)f = 0;
        m = 0;
        r = startmove();
      }
    }
  }
#ifndef ISPC

  // this both check take 8us
  temp_loop(cm);
#ifdef motortimeout
  if (!m   && (cm - nextmotoroff >= motortimeout)) {
    xprintf(PSTR("Motor off\n"));
    nextmotoroff = cm;
    power_off();
  }
#endif // motortimeout

#if defined(ESP8266)
  feedthedog();
#endif


#endif // ispc
}

/*
    =================================================================================================================================================
    STARTMOVE
    =================================================================================================================================================
  Mulai menjalankan 1 unit di buffer gerakan terakhir
*/
int subp = 1, laxis;

float ts;
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

  // convert from virtual step/mm value to original step/mm value
  stepdiv2 = wstepdiv2 * rampv;
  acup = oacup * rampv;
  acdn = oacdn * rampv;



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
  dln = xInvSqrt((int32_t)(m->fn ) << INVSCALE );
#ifdef SUBPIXELMAX
  int u;
  for (u = 2; u <= (SUBPIXELMAX); u++) {
    if (LOWESTDELAY * u > dln) {
      break;
    }
  }
  u--;
  subp = u;
  //zprintf(PSTR("Subpixel %d %d ->"),fi(u),fi(dln));
  if (u > 1) {
    totalstep *= u;
    acup /= u;
    acdn /= u;
    stepdiv2 /= u;
    stepdiv /= u;
    dln /= u;
#ifdef NONLINEAR
    rampv /= u;
#endif

  }
  //zprintf(PSTR(" %d\n"),fi(dln));
#endif
  //

  mctr = totalstep ;
  newdircommand();

#ifdef ISPC
  fctr = 0;
  m->col++;
#endif
}




int32_t startmove()
{

  if (cmdfull)return 0;
  if ((head == tail)) {
    //Serial.println("?");
    //m = 0; wm = 0; mctr = 0; // thi cause bug on homing delta
    return 0;
  }
  // last calculation
  if (m ) return 0; // if empty then exit
#ifdef output_enable
  //zprintf(PSTR("\nSTARTMOVE\n"));
#endif
  // STEP 1
  int t = nextbuff(tail);
  // prepare ramp (for last move)
  prepareramp(t);
  m = &move[t];

  laxis = fastaxis;
  fastaxis = FASTAXIS(m);
  totalstep = labs(m->dx[fastaxis]);


  // recalculate acceleration for up and down, to minimize rounding error
  acup = acdn = 0;
  //int acramp=((m->status & 8)?(accel):(mvaccel))/(AJERK);
  //m->rampdown-=acramp*2;
  if (m->rampup)acup = ((int32_t)(m->fn  - m->fs ) << INVSCALE ) / (float)m->rampup;//(fmax(1,m->rampup-acramp));
  if (m->rampdown)acdn = -((int32_t)(m->fn  - m->fe ) << INVSCALE ) / (float)m->rampdown;//(fmax(1,m->rampdown-acramp));
  oacup = acup;
  oacdn = acdn;

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
  ta = (((int32_t)(m->fs ) << INVSCALE)) ;
  stepdiv = CLOCKCONSTANT / (Cstepmmx(fastaxis));
  stepdiv2 = wstepdiv2 = stepdiv * INVSCALE2;
  m->status &= ~3;
  m->status |= 2;


  if (f == 0) nextmicros = micros();// if from stop

  rampup = m->rampup  ;
  rampdown = totalstep - rampup - m->rampdown ;
  mctr2 = mcx[0] = mcx[1] = mcx[2] = mcx[3] = 0; //mctr >> 1;
  tail = t;

#ifdef output_enable
  /*zprintf(PSTR("Start tail:%d head:%d\n"), fi(tail), fi(head));
    zprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), fi(rampup), fi(rampdown), fi(totalstep));
    zprintf(PSTR("FS:%f FN:%f FE:%f AC:%f \n"), ff(m->fs), ff(m->fn), ff(m->fe), ff(m->ac));
    zprintf(PSTR("TA,ACUP,ACDN:%d,%d,%d \n"), fi(ta), fi(acup), fi(acdn));
    zprintf(PSTR("DX:%d DY:%d DZ:%d DE:%d \n"), fi(m->dx[0]), fi(m->dx[1]), fi(m->dx[2]), fi(m->dx[3]));
  */
  //xprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
  //xprintf(PSTR("sx %d %d %d \n"), fi(sx[0]), fi(sx[1]), fi(sx[2]));
  //xprintf(PSTR("Status:%d \n"), fi(m->status));

#endif
  calculate_delta_steps();
#ifdef SUBPIXELMAX
  rampup *= subp;
  rampdown *= subp;
#ifdef output_enable
  zprintf(PSTR("SUB:%d RAMP:%d %d ACC:%d %d\n"), fi(subp), fi(rampup), fi(rampdown), fi(acup), fi(acdn));
#endif
#endif
  dlp = xInvSqrt(ta);
  return 1;
}

/*
    =================================================================================================================================================
    DOCHECKENDSTOP
    =================================================================================================================================================
*/
#ifdef INVERTENDSTOP
#define ENDCHECK !
#else
#define ENDCHECK
#endif

#define ENDPIN INPUT_PULLUP

void docheckendstop()
{

  // AVR specific code here
  // read end stop



#ifndef ISPC

#ifdef limit_pin
  int nc = 0;
  endstopstatus=0;
  for (int d = 0; d < 3; d++) {
    if (ENDCHECK dread(limit_pin))nc++;
  }
  if (nc > 1)endstopstatus = -1;
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
void homing()
{
  // clear buffer
  addmove(100, 0, 0, cz1, ce01);
  waitbufferempty();
  ishoming = 1;
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  ce01 = 0;

  x2[0] = x2[1] = x2[2] = x2[3] = 0;

  int32_t stx[NUMAXIS];
//  stx[0] =  stx[1] = stx[2] = stx[3] = 0;
  int32_t tx[NUMAXIS];
//  tx[0] =  tx[1] = tx[2] = tx[3] = 0;
#define mmax HOMING_MOVE
#define smmax ENDSTOP_MOVE
  int vx[4] = { -1, -1, -1, -1};
  for (int t = 0; t < NUMAXIS; t++) {
    if (ax_home[t] > 1)vx[t] = 1;
    if (ax_home[t] < 1)vx[t] = 0;
    tx[t] = mmax * vx[t];
    stx[t] = smmax * vx[t];
  }

  // fast check every 31 step
  //zprintf(PSTR("Homing to %d %d %d\n"), tx[0], tx[1], tx[2]);

  // move away before endstop
#ifdef DRIVE_XYYZ
  checkendstop = 0;
  addmove(homingspeed, -stx[0], -stx[1], -stx[2], -stx[3], 1, 1);
  waitbufferempty();

  checkendstop = 31;
  addmove(homingspeed, 0, tx[1], tx[2], 0, 1, 1);
  waitbufferempty();
  checkendstop = 0;
  addmove(homingspeed, 0, -stx[1], -stx[2], 0, 1, 1);
  waitbufferempty();
  checkendstop = 31;
  addmove(homingspeed, tx[0], 0, 0, tx[3], 1, 1);
#else
  checkendstop = 31;
  addmove(homingspeed, tx[0], tx[1], tx[2], tx[3], 1, 1);
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
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    checkendstop = 1; \
    waitbufferempty(); \
    xx[e] =  - stx[e] - axisofs[e]; \
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    checkendstop = 0; \
    waitbufferempty(); \
  }
  for (int e = 0; e < NUMAXIS; e++) {
    moveaway(e, homingspeed);
  }
  for (int e = 0; e < NUMAXIS; e++) {
    if (vx[e]) {
      // check endstop again fast
      xcheckendstop(e, 25);
      xcheckendstop(e, 15);
    }
  }
  checkendstop = ce01 = 0;

#ifdef NONLINEAR

  NONLINEARHOME

#else // NONLINEAR

  if (vx[0] > 0) cx1 = ax_home[0]; else  cx1 = 0;
  if (vx[1] > 0) cy1 = ax_home[1]; else  cy1 = 0;
  if (vx[2] > 0) cz1 = ax_home[2]; else  cz1 = 0;

#endif // NONLINEAR

  //xprintf(PSTR("Home to:X:%f Y:%f Z:%f\n"),  ff(cx1), ff(cy1), ff(cz1));
  ishoming = 0;
  init_pos();

}

/*
    =================================================================================================================================================
    WAITBUFFEREMPTY
    =================================================================================================================================================
  waitbufferempty
  Proses semua gerakan di buffer, dan akhiri dengan deselerasi ke 0
*/
void waitbufferempty()
{
  if (head != tail)prepareramp(head);
  startmove();
#ifdef output_enable
  zprintf(PSTR("Wait %d\n"), fi(mctr));
#endif
  MEMORY_BARRIER()
  LOOP_IN(2)
  domotionloop
  while ((head != tail) || m) { //(tail == otail) //

    domotionloop
    servo_loop();
    MEMORY_BARRIER()
    //zprintf(PSTR("->%d\n"), fi(mctr));
  }
  LOOP_OUT(2)
#ifdef output_enable
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
    xprintf(PSTR("Wait %d / %d \n"), fi(tail), fi(head));
#endif
    //wait current move finish
    int t = tail;
    LOOP_IN(3)
    MEMORY_BARRIER()
    while (t == tail) {
      domotionloop
      servo_loop();
      MEMORY_BARRIER()
      //zprintf(PSTR("%d\n"), fi(mctr));
    }
    //xprintf(PSTR("Done\n"));
    LOOP_OUT(1)

#ifdef output_enable
    xprintf(PSTR("Done %d / %d \n"), fi(tail), fi(head));
#endif
  }

}
/*
    =================================================================================================================================================
    initmotion
    =================================================================================================================================================
  inisialisasi awal, wajib
*/
void init_pos()
{
}
void initmotion()
{
#ifdef __ARM__
  disableDebugPorts();
#endif
  reset_motion();
  preparecalc();
  dl = 5000;

#ifdef ISPC
  tickscale = 60;
  fscale = 2;
  graphscale = 4;
#endif
  motor_0_INIT();
  motor_1_INIT();
  motor_2_INIT();
  motor_3_INIT();

#ifndef ISPC
#ifdef limit_pin
  pinMode(limit_pin, ENDPIN);
#endif
  pinMotorInit
#endif

}
