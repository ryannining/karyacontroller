#include "motion.h"
#include "common.h"
#include "timer.h"
#include "config_pins.h"
#include "temp.h"
#include <stdint.h>

#if defined(__AVR__)
#include<arduino.h>
#elif defined(ESP8266)
#include<arduino.h>
#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#endif



int32_t homingspeed;
float homeoffset[4] ;
int32_t jerk[4] ;
int32_t accel[4];
int32_t mvaccel[4];
int32_t maxf[4];
float stepmmx[4];
int8_t checkendstop;
int8_t endstopstatus[3];
int32_t head, tail;
tmove move[NUMBUFFER];
float cx1, cy1, cz1, ce01, lf;
float ax_max[3];


static int32_t ramplen(float v0, float v1, float a , float stepmm)
{
  float t = (v1 - v0) / a;
  return fabs((v0 * t + 0.5 * a * t * t) * stepmm);
}
static float speedat(float v0, float a, float s, float stp)
{
  return sqrt(a * 2 * s / stp + v0 * v0);
}
static float accelat(float v0, float v1, float s)
{
  //v1=sqr(a*2*s+v0*v0)
  //a=(v1*v1-v0*v0)/(2*s)
  return (v1 * v1 - v0 * v0) / (2 * s);
}

/*
    =================================================================================================================================================
    RESET_MOTION
    =================================================================================================================================================
  Reset all variable
*/
void reset_motion() {
  homingspeed = HOMINGSPEED;
  homeoffset[0] = XOFFSET;
  homeoffset[1] = YOFFSET;
  homeoffset[2] = ZOFFSET;
  homeoffset[3] = EOFFSET;

  maxf[0] = XMAXFEEDRATE;
  maxf[1] = YMAXFEEDRATE;
  maxf[2] = ZMAXFEEDRATE;
  maxf[3] = E0MAXFEEDRATE;


  jerk[0] = XJERK;
  jerk[1] = YJERK;
  jerk[2] = ZJERK;
  jerk[3] = E0JERK;

  accel[0] = XACCELL;
  accel[1] = YACCELL;
  accel[2] = ZACCELL;
  accel[3] = E0ACCELL;

  mvaccel[0] = XMOVEACCELL;
  mvaccel[1] = YMOVEACCELL;
  mvaccel[2] = ZMOVEACCELL;
  mvaccel[3] = E0ACCELL;

  stepmmx[0] = XSTEPPERMM;
  stepmmx[1] = YSTEPPERMM;
  stepmmx[2] = ZSTEPPERMM;
  stepmmx[3] = E0STEPPERMM;

  ax_max[0] = XMAX;
  ax_max[1] = YMAX;
  ax_max[2] = ZMAX;

  checkendstop = 0;
  endstopstatus[0] = 0;
  endstopstatus[1] = 0;
  endstopstatus[2] = 0;
  head = tail = 0;
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  ce01 = 0;
  tick = 0;

}

int32_t mcx[NUMAXIS];
/*
    =================================================================================================================================================
    MOTOR CLASS
    =================================================================================================================================================
*/

#define DUMMYMOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){}\
  inline void motor_##AX##_ON(){}\
  inline void motor_##AX##_OFF() {}\
  inline void motor_##AX##_DIR(int d){}\
  inline void motor_##AX##_STEP(){}\
  inline void motor_##AX##_UNSTEP(){}

#if defined(__AVR__) || defined(ESP8266)

#define MOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){pinMode(PENABLE, OUTPUT);pinMode(PDIR, OUTPUT);pinMode(PSTEP, OUTPUT);digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_ON(){ digitalWrite(PENABLE,0);}\
  inline void motor_##AX##_OFF() { digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_DIR(int d){ digitalWrite(PENABLE,0);digitalWrite(PDIR,d>0?1:0);}\
  inline void motor_##AX##_STEP(){  digitalWrite(PSTEP,1);}\
  inline void motor_##AX##_UNSTEP(){  digitalWrite(PSTEP,0);}

#else
#define MOTOR(AX,PENABLE,PDIR,PSTEP) DUMYMOTOR(AX,0,0,0)
#endif


#ifdef xenable
MOTOR(0, xenable, xdirection, xstep)
#else
DUMMYMOTOR(0, 0, 0, 0)
#endif

#ifdef yenable
MOTOR(1, yenable, ydirection, ystep)
#else
DUMMYMOTOR(1, 0, 0, 0)
#endif

#ifdef zenable
MOTOR(2, zenable, zdirection, zstep)
#else
DUMMYMOTOR(2, 0, 0, 0)
#endif

#ifdef e0enable
MOTOR(3, e0enable, e0direction, e0step)
#else
DUMMYMOTOR(3, 0, 0, 0)
#endif

int mb_ctr;
int32_t bufflen() {
  mb_ctr = head >= tail ? head - tail : (NUMBUFFER + head) - tail; // 5+0 - 4
  return mb_ctr;
}

void power_off() {
  motor_0_OFF();
  motor_1_OFF();
  motor_2_OFF();
  motor_3_OFF();

}
/*
    =================================================================================================================================================
    SAFESPEED
    =================================================================================================================================================
  Bertujuan mencari kecepatan aman dari sebuah gerakan, misalnya gerakan dg speed 100, dan max speed 50, maka perlu diskala 0.5

*/
void safespeed(tmove *m) {
  int32_t i;

  float  scale = 1;

  for (i = 0; i < NUMAXIS; i++) {
    if (m->dx[i] > 0) {
      m->fx[i] = m->fn * m->dx[i] / m->totalstep;
      //print32_t .fx(i)

      //xprintf(PSTR("MF:%f F:%f\n"),ff((float)maxf[i]),ff((float)m->fx[i]));
      float scale2 = (float)maxf[i] / (float)m->fx[i];
      if (scale2 < scale) scale = scale2;
    }
  }
  // update all speed

  m->fn = m->fn * scale;
  //xprintf(PSTR("SCALE:%f\n"),ff(scale));
  for (i = 0; i < NUMAXIS; i++) {
    m->fx[i] = m->fx[i] * scale * m->sx[i];
    //xprintf(PSTR("F%d:%f\n"),i,ff((float)m->fx[i]));
  }
  //print32_t "w",.fx(1),.fx(2),.fx(3)
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

void prepareramp(int32_t bpos)
{
  tmove *m;
  m = &move[bpos];
  //color .col
  if (m->planstatus == 1)return;
  //print32_t bpos

  float t, ac;
  float stepmm = stepmmx[m->fastaxis];
  if (m->g0)  ac = mvaccel[m->fastaxis]; else ac = accel[m->fastaxis];
  m->ac1 = ACCELL(m->fs, m->fn, ac);
  m->ac2 = ACCELL(m->fn, m->fe, ac);
  m->rampup = ramplen(m->fs, m->fn, m->ac1, stepmm);
  m->rampdown = ramplen(m->fe, m->fn, m->ac2, stepmm);

  //#define preprampdebug

#ifdef preprampdebug
  xprintf(PSTR("Bpos:%d\n"), (int32_t)m->bpos);
  xprintf(PSTR("----------1-----------\nRU:%d Rld:%d TS:%d\n "), m->rampup, m->rampdown, m->totalstep);
  xprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
  // New generic algorithm, to calculate rampup and rampdown simpler, and faster
  // if rampup and ramp down overlap
  if (m->rampup + m->rampdown > m->totalstep) {
    int r = ((m->rampdown + m->rampup) - m->totalstep) / 2;
    m->rampup -= r;
    m->rampdown -= r;
    // check if still larger that totalstep
    if (m->rampup > m->totalstep) {
      // adjust speed
      m->fn = speedat(m->fs, m->ac1, m->totalstep, stepmm);
      m->fe = m->fn;
      m->rampdown = 0;
      m->rampup = m->totalstep;
#ifdef preprampdebug
      xprintf(PSTR("========2========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      xprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else if (m->rampdown > m->totalstep) {
      // adjust acceleration
      m->ac2 = accelat(m->fs, m->fe, m->totalstep) * stepmm;
      m->rampup = 0;
      m->rampdown = m->totalstep;
#ifdef preprampdebug
      xprintf(PSTR("========3========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      xprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else {
      m->fn = speedat(m->fs, m->ac1, m->rampup, stepmm);

    }
  }


#ifdef preprampdebug
  xprintf(PSTR("========FINAL========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
  xprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
#endif
  m->planstatus = 1;
  m->ac1 *= stepmmx[m->fastaxis] * TMSCALE / timescale ;
  m->ac2 *= stepmmx[m->fastaxis] * TMSCALE / timescale ;
  if (m->fs < 1) m->fs = 1;
  m->fs *= stepmmx[m->fastaxis] * TMSCALE;
  //zprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
}

/*
    =================================================================================================================================================
    PLANNER
    =================================================================================================================================================
  dipanggil oleh   untuk mulai menghitung dan merencakanan gerakan terbaik
  kontrol jerk adalah cara mengontrol supaya saat menikung, kecepatan dikurangi sehingga tidak skip motornya
  jerk sendiri dikontrol per axis oleh variabel
*/
void planner(int32_t h)
{
  // mengubah semua cross feedrate biar optimal secepat mungkin
  int32_t i, p;
  tmove *curr;
  tmove *prev;


  curr = &move[h];
  safespeed(curr);
  if (bufflen() < 2) return;
  p = prevbuff(h);
  prev = &move[p];
  if (prev->status == 2)return;
  /* calculate jerk
                max_jerk
           x = -----------
                |v1 - v2|

           if x > 1: continue full speed
           if x < 1: v = v_max * x
  */
  float scale, scale2;
  float cjerk = jerk[curr->fastaxis];

  scale = 1;
  for (i = 0; i < NUMAXIS; i++) {
    float df = fabs(curr->fx[i] - prev->fx[i]);
    //xprintf("DFX%d:%f %f\n",i,prev->fx[i],curr->fx[i]);
    scale2 = cjerk / df;
    if (scale2 < scale)  scale = scale2;
  }
  //xprintf ("jerk scaling:%f\n",scale);

  if ((curr->status == 1) && (prev->status != 0)) {
    //remove prev rampdown, curr rampup
    prev->fe = scale * (prev->fn + curr->fn) / 2; //curr->fn; //
    //print32_t *(prev).fe,scale
    //*(prev).fe=scale*(*(curr).fn)
    prepareramp(p);
    //update current move again
    curr->fs = prev->fe;
  }

}

/*
    =================================================================================================================================================
    ADDMOVE
    =================================================================================================================================================
  Rutin menambahkan sebuah vektor ke dalam buffer gerakan
*/
int32_t x1[NUMAXIS], x2[NUMAXIS];
void addmove(float cf, float cx2, float cy2 , float cz2, float ce02, int g0 , int rel)
{
  //cf=100;
#ifdef output_enable
  xprintf(PSTR("Tail:%d Head:%d \n"), tail, head);
  xprintf(PSTR("From X:%f Y:%f Z:%f\n"), ff(cx1), ff(cy1), ff(cz1));
  xprintf(PSTR("to F:%f X:%f Y:%f Z:%f\n"), ff(cf), ff(cx2), ff(cy2), ff(cz2));
#endif
  //zprintf(PSTR("X:%f Y:%f Z:%f\n"), ff(cx2), ff(cy2), ff(cz2));
  //zprintf(PSTR("-\n"));
  needbuffer();
  tmove *am;
  am = &move[nextbuff(head)];
  am->g0 = g0;
  am->col = 2 + (head % 2) * 8;
  x1[0] = cx1 * stepmmx[0];
  x1[1] = cy1 * stepmmx[1];
  x1[2] = cz1 * stepmmx[2];
  x1[3] = ce01 * stepmmx[3];

  x2[0] = cx2 * stepmmx[0];
  x2[1] = cy2 * stepmmx[1];
  x2[2] = cz2 * stepmmx[2];
  x2[3] = ce02 * stepmmx[3];
#ifdef ISPC
  am->xs[0] = x1[0];
  am->xs[1] = x1[1];
#endif
  am->fn = cf;
  am->fe = 0;
  am->fs = 0;
  am->planstatus = 0; //0: not optimized 1:fixed
  am->status = 1; // 0: finish 1:ready 2:running
  //calculate delta
  int32_t ix;
  for (ix = 0; ix < NUMAXIS; ix++) {

    int32_t delta = rel ? x2[ix] : (x2[ix] - x1[ix]);
    am->dx[ix] = abs(delta);
    am->sx[ix] = 0;
    if (delta > 0) am->sx[ix] = 1;
    if (delta < 0) am->sx[ix] = -1;

    //xprintf(PSTR("Dx%d:%d\n"), fi(ix), fi(am->dx[ix]));
    //xprintf(PSTR("Sx%d:%d\n"), fi(ix), fi(am->sx[ix]));
  }
  for (ix = 0; ix < NUMAXIS; ix++) {
    if ((am->dx[ix] >= am->dx[0])
        && (am->dx[ix] >= am->dx[1])
        && (am->dx[ix] >= am->dx[2])
        && (am->dx[ix] >= am->dx[3])
       )  {
      am->fastaxis = ix;
    }
  }
  am->totalstep = am->dx[am->fastaxis];
#ifdef output_enable

  xprintf(PSTR("Totalstep AX%d %d\n"), (int32_t)am->fastaxis, (int32_t)am->totalstep);
#endif
  // } buffer please

  // back planner
  head = nextbuff(head);
  am->bpos = head;
  planner(head);
#define realmove(i)am->sx[i]*am->dx[i]/stepmmx[i]

#define subpixel
#ifdef subpixel
  cx1 = /*cx2;*/ cx1 + realmove(0);
  cy1 = /*cy2;/*/cy1 + realmove(1);
  cz1 = /*cz2;/*/cz1 + realmove(2);
  ce01 = /*ce02;/*/ce01 + realmove(3);
#else
  cx1 = cx2;
  cy1 = cy2;
  cz1 = cz2;
  ce01 = ce02;
#endif
  am->status = 1; // 0: finish 1:ready 2:running
}


tmove *m;
float tick, tickscale, fscale, graphscale;
//int32_t px[4] = {0, 0, 0, 0 };
int32_t ac1, ac2, f, dl;
int32_t mctr;
uint32_t nextmicros;
uint32_t nextdly;
uint32_t nextmotoroff;


/*
    =================================================================================================================================================
    MOTIONLOOP
    =================================================================================================================================================
*/
uint32_t ocm;
int motionloop() {
  uint32_t cm, ix;
#if defined(ESP8266)
  feedthedog();
#endif
#ifndef ISPC
  cm = micros();
  temp_loop(cm);
  if (cm - nextmicros >= motortimeout) {
    //xprintf(PSTR("Motor off\n"));
    nextmotoroff = cm + motortimeout;
    power_off();
  }
#endif
  if (!m ) {
    // start new move if available , if not exit
    if (!startmove()) return 0;
  }
  //  if (m->status == 2) {
  if (mctr >= m->totalstep) {
    //xprintf(PSTR("Finish:%d\n"),mctr);
    m->status = 0;
    m = 0;
    return 0;
  }
#ifdef ISPC
  if (1) {
#else
  if (cm - nextmicros >= dl) {
#endif

    // update delay not every step to make the motor move faster
    if (m->g0) {
#ifdef UPDATE_F_EVERY
      nextdly += dl;
      if (nextdly > UPDATE_F_EVERY)
      {
        nextdly -= UPDATE_F_EVERY;
        dl = timescaleLARGE / f;
      }
#else
      dl = timescaleLARGE / f;
#endif
    } else dl = timescaleLARGE / f;

#ifdef ISPC
    tick = tick + dl;//1.0/timescale;
    int32_t c = m->col;

    //pset (tick*tickscale+10,400-f*fscale),c
    float cf = float(timescale);
    cf /= dl * stepmmx[m->fastaxis];
#define realpos1(i) (m->xs[i] / stepmmx[i])
    putpixel (tick * tickscale / timescale + 30, 450 -  fscale * cf, c - 8);
    putpixel (tick * tickscale / timescale + 10, 400 -  fscale * f / TMSCALE / stepmmx[m->fastaxis], c);
    putpixel (graphscale * realpos1(0) + 250, graphscale * realpos1(1) + 240, c);
    //zprintf(PSTR("%f\n"), cf);
    //pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
#endif


#ifdef ISPC
#define graphstep(ix) m->xs[ix] +=m->sx[ix];
#else
#define graphstep(ix)
#endif
    /* ================================================================================================================
                                                    BRESENHAM CORE

                                  this need to move to interrupt for smooth movement
       ================================================================================================================
    */

#define bresenham(ix)\
  if ((mcx[ix] -= m->dx[ix]) < 0) {\
    motor_##ix##_STEP();\
    mcx[ix] += m->totalstep;\
    graphstep(ix) \
  }

#ifdef e0step
    bresenham(3)
    motor_3_UNSTEP();
#endif
    bresenham(0);
    bresenham(1);
    bresenham(2);
    motor_0_UNSTEP();
    motor_1_UNSTEP();
    motor_2_UNSTEP();
    // next speed
    if (mctr < m->rampup) {
      f += (ac1 * dl);
    }
    else if (mctr > m->rampdown ) // already subtracted from totalstep
    {
      f += (ac2 * dl);
    }
    mctr++;
    nextmicros = cm;
    /* ================================================================================================================ */
    //if (m->bpos==8)
    //if (mctr % 60 == 0)zprintf(PSTR("F:%f D:%d\n"), ff(f/ stepmmx[m->fastaxis]),dl);

    if (checkendstop) {
      docheckendstop();
      for (int32_t e = 0; e < 3; e++)
      {
        if (endstopstatus[e] < 0) {
          m->status = 0;
          m = 0;
          checkendstop = 0;
          return 1;
        }
      }
    }
  }
  return 1;
  //  }
  //  return 0;
}

/*
    =================================================================================================================================================
    STARTMOVE
    =================================================================================================================================================
  Mulai menjalankan 1 unit di buffer gerakan terakhir
*/
int32_t startmove()
{
  if (m) exit;
  if (head != tail) { // if empty then exit
    int32_t t = nextbuff(tail);
    int ix;
    m = &move[t];
    if (m->status == 1) {

      nextmicros = micros();
      motor_0_DIR(m->sx[0]);
      motor_1_DIR(m->sx[1]);
      motor_2_DIR(m->sx[2]);
      motor_3_DIR(m->sx[3]);
      mcx[0] = mcx[1] = mcx[2] = mcx[3] = (m->totalstep / 2);
      prepareramp(t);
#ifdef output_enable
      xprintf(PSTR("Start buff:%d\n"), tail);
      xprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), m->rampup, m->rampdown, m->totalstep);
      xprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
      //xprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
      xprintf(PSTR("sx %d %d %d \n"), fi(m->sx[0]), fi(m->sx[1]), fi(m->sx[2]));
#endif
      tail = t;
      m->status = 2;
      f = m->fs;// m->fs*= stepmmx[m->fastaxis]; in planner
      dl = timescaleLARGE / f;
      ac1 = m->ac1;
      ac2 = m->ac2;
      m->rampdown = m->totalstep - m->rampdown; // presubtract
      nextdly = 0;
      mctr = 0;
      return 1;
    }

  }
  return 0;
}

/*
    =================================================================================================================================================
    DOCHECKENDSTOP
    =================================================================================================================================================
*/
void docheckendstop()
{
  // AVR specific code here
  // read end stop
#ifdef xmin_pin
  endstopstatus[0] = xmin_pin;
#elif defined(xmax_pin)
  endstopstatus[0] = xmin_pin;
#else
  endstopstatus[0] = 0;
#endif
#ifdef ymin_pin
  endstopstatus[1] = ymin_pin;
#elif defined(ymax_pin)
  endstopstatus[1] = ymax_pin;
#else
  endstopstatus[1] = 0;
#endif
#ifdef zmin_pin
  endstopstatus[2] = zmin_pin;
#elif defined(zmax_pin)
  endstopstatus[2] = zmax_pin;
#else
  endstopstatus[2] = 0;
#endif


#if defined(__AVR__)
  for (int32_t e = 0; e < 3; e++) {
    if (endstopstatus[e]) {
      endstopstatus[e] = -digitalRead(endstopstatus[e]);
    }
  }
#elif defined(ESP8266)
  // ESP8266 specific code here
  // read end stop

#else
  // PC
  if (!m) return;
  // simulate endstop if x y z is 0
  for (int32_t e = 0; e < NUMAXIS; e++) {
    if (x1[e] < 0) endstopstatus[e] = -1; else  endstopstatus[e] = 0;
  }
#endif
}

/*
    =================================================================================================================================================
    HOMING
    =================================================================================================================================================
*/
void homing(float x, float y, float z, float e0)
{
  // clear buffer
  waitbufferempty();
  int32_t tx[4];
  tx[0] =  tx[1] = tx[2] = 0;
#ifdef xmin_pin
  tx[0] = -500;
#elif defined(xmax_pin)
  tx[0] = 500;
#endif

#ifdef ymin_pin
  tx[1] = -500;
#elif defined(ymax_pin)
  tx[1] = 500;
#endif
#ifdef zmin_pin
  tx[2] = -500;
#elif defined(zmax_pin)
  tx[2] = 500;
#endif
  checkendstop = 1;
  //xprintf(PSTR("Homing to %d %d %d\n"), tx[0], tx[1], tx[2]);
  addmove(homingspeed, tx[0], tx[1], tx[2], ce01);

  // now slow down and check endstop once again
  waitbufferempty();
  //docheckendstop();
  float xx[4];
  //xprintf(PSTR("ENDSTOP %d %d %d\n"), (int32_t)endstopstatus[0], (int32_t)endstopstatus[1], (int32_t)endstopstatus[2]);
  //xprintf(PSTR("Position %f %f %f \n"), ff(cx1), ff(cy1), ff(cz1));
  // move away from endstop
#define moveaway(e,F) {\
    if (tx[e]) {\
      xx[0]=xx[1]=xx[2]=xx[3]=0;\
      xx[e] =  - tx[e] / 100;\
      checkendstop = 0;\
      addmove(F, xx[0], xx[1], xx[2], 0,1,1);\
      waitbufferempty();\
    }\
  }
#define checkendstop(e,F) {\
    xx[0]=xx[1]=xx[2]=0;\
    xx[e] = tx[e];\
    checkendstop = 1;\
    addmove(F, xx[0], xx[1], xx[2], 0,1,1);\
    waitbufferempty();\
    xx[e] =  - tx[e] / 200;\
    checkendstop = 0;\
    addmove(F, xx[0], xx[1], xx[2], 0,1,1);\
    waitbufferempty();\
  }
  for (int32_t e = 0; e < 3; e++) {
    moveaway(e, homingspeed);
  }
  for (int32_t e = 0; e < 3; e++) {
    if (tx[e]) {
      // check endstop again fast
      checkendstop(e, homingspeed);
      checkendstop(e, homingspeed / 10);
    }
  }
  checkendstop = ce01 = 0;

#ifdef xmax_pin
  cx1 = ax_max[0];
#else
  cx1 = 0;
#endif

#ifdef ymax_pin
  cy1 = ax_max[1];
#else
  cy1  = 0;
#endif

#ifdef zmax_pin
  cz1 = ax_max[2];
#else
  cz1 = 0;
#endif
  ce01 = 0;
  //xprintf(PSTR("Home to:X:%f Y:%f Z:%f\n"),  ff(cx1), ff(cy1), ff(cz1));

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
  startmove();
  while ((head != tail) || m) {
    motionloop();
  }
}
/*
    =================================================================================================================================================
    NEEDBUFFER
    =================================================================================================================================================
  loop sampai da buffer yang bebas
*/
void needbuffer()
{
#ifdef output_enable
  xprintf(PSTR("buffer %d / %d \n"), tail, head);
#endif
  if (nextbuff(head) == tail) {
#ifdef output_enable
    xprintf(PSTR("Wait\n"));
#endif
    while (m) {
      motionloop();
    }
    startmove();
    int lt = tail;
    while (m) {
      motionloop();
    }
    m = 0;
    //xprintf(PSTR("Done\n"));
  }
}
/*
    =================================================================================================================================================
    initmotion
    =================================================================================================================================================
  inisialisasi awal, wajib
*/
void initmotion() {

  reset_motion();
  tickscale = 60;
  fscale = 2;
  graphscale = 4;
  int32_t i;
  motor_0_INIT();
  motor_1_INIT();
  motor_2_INIT();
  motor_3_INIT();

#if defined( __AVR__) || defined(ESP8266)
#ifdef xmin_pin
  pinMode(xmin_pin, INPUT_PULLUP);
#endif
#ifdef xmax_pin
  pinMode(xmax_pin, INPUT_PULLUP);
#endif

#ifdef ymin_pin
  pinMode(ymin_pin, INPUT_PULLUP);
#endif
#ifdef ymax_pin
  pinMode(ymax_pin, INPUT_PULLUP);
#endif

#ifdef zmin_pin
  pinMode(zmin_pin, INPUT_PULLUP);
#endif
#ifdef zmax_pin
  pinMode(zmax_pin, INPUT_PULLUP);
#endif

  /*
    Serial.print(digitalRead(13));
    zprintf(PSTR("Z:%d"),(int32_t)digitalRead(13));
    zprintf(PSTR("Y:%d"),(int32_t)digitalRead(A0));
    zprintf(PSTR("X:%d\n"),(int32_t)digitalRead(A1));
  */
#endif
}



