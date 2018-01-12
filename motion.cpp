







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
float cx1, cy1, cz1, ce01, lf, e_multiplier;
float ax_max[3];
int xback[4];
int32_t dl;


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

int8_t lsx[4] = {0, 0, 0, 0};

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
  lsx[0] = 0;
  lsx[1] = 0;
  lsx[2] = 0;
  lsx[3] = 0;
  xback[0] = MOTOR_X_BACKLASH;
  xback[1] = MOTOR_Y_BACKLASH;
  xback[2] = MOTOR_Z_BACKLASH;
  xback[3] = MOTOR_E_BACKLASH;

  e_multiplier = 1;

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

//    zprintf(PSTR("Backlash AX%d %d\n"),fi(AX),fi(PSTEP));\

#define MOTORBACKLASH(AX,d,PSTEP) \
  if (PSTEP && lsx[AX] && (lsx[AX]!=d)){\
    for(int i=0;i<PSTEP;i++){\
      motor_##AX##_STEP();\
      delayMicroseconds(5);\
      motor_##AX##_UNSTEP();\
      delayMicroseconds(dl);\
    }\
  }\
  lsx[AX]=d;\



#define MOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){pinMode(PENABLE, OUTPUT);pinMode(PDIR, OUTPUT);pinMode(PSTEP, OUTPUT);digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_ON(){ digitalWrite(PENABLE,0);}\
  inline void motor_##AX##_OFF() { digitalWrite(PENABLE,1);}\
  inline void motor_##AX##_STEP(){  digitalWrite(PSTEP,1);}\
  inline void motor_##AX##_UNSTEP(){  digitalWrite(PSTEP,0);}\
  inline void motor_##AX##_DIR(int d){ if(!d)return;digitalWrite(PENABLE,0);digitalWrite(PDIR,(d*MOTOR_##AX##_DIR)>0?1:0);MOTORBACKLASH(AX,d,xback[AX]);}\

#else
#define MOTOR(AX,PENABLE,PDIR,PSTEP)\
  inline void motor_##AX##_INIT(){zprintf(PSTR("Motor ##AX## Init\n"));}\
  inline void motor_##AX##_ON(){ zprintf(PSTR("Motor ##AX## Enable\n"));}\
  inline void motor_##AX##_OFF() { zprintf(PSTR("Motor ##AX## Disable\n"));}\
  inline void motor_##AX##_STEP(){  zprintf(PSTR("Motor ##AX## step\n"));}\
  inline void motor_##AX##_UNSTEP(){  zprintf(PSTR("Motor ##AX## unstep"\n));}\
  inline void motor_##AX##_DIR(int d){ zprintf(PSTR("Motor ##AX## Dir %d Backlash %d\n"),d,MOTOR_##AX##_BACKLASH);}\


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
  if (m->status & 4)return; // already calculated
  //print32_t bpos

  float t, ac;
  float stepmm = stepmmx[m->fastaxis];
  if (m->status & 8)  ac = mvaccel[m->fastaxis]; else ac = accel[m->fastaxis];
  m->ac1 = ACCELL(m->fs, m->fn, ac);
  m->ac2 = ACCELL(m->fn, m->fe, ac);
  m->rampup = ramplen(m->fs, m->fn, m->ac1, stepmm);
  m->rampdown = ramplen(m->fe, m->fn, m->ac2, stepmm);

  //#define preprampdebug

#ifdef preprampdebug
  zprintf(PSTR("Bpos:%d\n"), (int32_t)bpos);
  zprintf(PSTR("----------1-----------\nRU:%d Rld:%d TS:%d\n "), m->rampup, m->rampdown, m->totalstep);
  zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
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
      zprintf(PSTR("========2========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else if (m->rampdown > m->totalstep) {
      // adjust acceleration
      m->ac2 = accelat(m->fs, m->fe, m->totalstep) * stepmm;
      m->rampup = 0;
      m->rampdown = m->totalstep;
#ifdef preprampdebug
      zprintf(PSTR("========3========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else {
      m->fn = speedat(m->fs, m->ac1, m->rampup, stepmm);

    }
  }


#ifdef preprampdebug
  zprintf(PSTR("========FINAL========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
  zprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
#endif
  m->ac1 *= stepmmx[m->fastaxis] * TMSCALE / timescale ;
  m->ac2 *= stepmmx[m->fastaxis] * TMSCALE / timescale ;
  if (m->fs < 1) m->fs = 1;
  m->fs *= stepmmx[m->fastaxis] * TMSCALE;
  m->status |= 4;
  m->rampdown = m->totalstep - m->rampdown; // presubtract
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
  if (bufflen() < 1) return;
  p = prevbuff(h);
  prev = &move[p];
  if ((prev->status & 3) == 2)return;
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

  if (prev->status & 3) {
    //zprintf(PSTR("Backplanner\n"));
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
  //cf = 30;
  tmove *am;
  am = &move[nextbuff(head)];
  am->status = g0 ? 8 : 0; // reset status:0 planstatus:0 g0:g0
  //am->g0 = g0;
#ifdef ISPC
  am->col = 2 + (head % 2) * 8;
  am->xs[0] = cx1 * stepmmx[0];
  am->xs[1] = cy1 * stepmmx[1];
#endif

  if (rel) {
    cx1 = cy1 = cz1 = ce01 = 0;
  }

  // deltas
  x2[0] = (int32_t)(cx2 * stepmmx[0]) - (int32_t)(cx1 * stepmmx[0]) ;
  x2[1] = (int32_t)(cy2 * stepmmx[1]) - (int32_t)(cy1 * stepmmx[1]) ;
  x2[2] = (int32_t)(cz2 * stepmmx[2]) - (int32_t)(cz1 * stepmmx[2]) ;
  x2[3] = (int32_t)(ce02 * stepmmx[3]) - (int32_t)(ce01 * stepmmx[3]) ;
  x2[3] *= e_multiplier;
#if defined( DRIVE_COREXY)
  // 2 = z axis + xy H-gantry (x_motor = x+y, y_motor = y-x)
  // borrow cx1,cy1 for calculation
  cx1 = x2[0] + x2[1];
  cy1 = x2[1] - x2[0];
  x2[0] = cx1;
  x2[1] = cy1;

#elif defined(DRIVE_COREXZ)
  // 8 = y axis + xz H-gantry (x_motor = x+z, z_motor = x-z)
  cx1 = x2[0] + x2[2];
  cy1 = x2[0] - x2[2];
  x2[0] = cx1;
  x2[2] = cy1;
#endif

  am->fn = cf;
  am->fe = 0;
  am->fs = 0;
  //am->planstatus = 0; //0: not optimized 1:fixed
  //calculate delta
  int32_t dd, ix;
  dd = 0;
  am->fastaxis = 0;
  for (ix = 0; ix < NUMAXIS; ix++) {
    int32_t delta = x2[ix];
    am->dx[ix] = abs(delta);
    am->sx[ix] = 0;
    if (delta > 0) am->sx[ix] = 1;
    if (delta < 0) am->sx[ix] = -1;
    if (am->dx[ix] > dd) {
      dd = am->dx[ix];
      am->fastaxis = ix;
    }
  }
  am->totalstep = am->dx[am->fastaxis];
#ifdef output_enable
  xprintf(PSTR("Totalstep AX%d %d\n"), (int32_t)am->fastaxis, (int32_t)am->totalstep);
#endif

  // back planner
  head = nextbuff(head);
  am->status |= 1; // 0: finish 1:ready
  planner(head);
  cx1 = cx2;
  cy1 = cy2;
  cz1 = cz2;
  ce01 = ce02;
}


tmove *m;
float tick, tickscale, fscale, graphscale;
//int32_t px[4] = {0, 0, 0, 0 };
int32_t ac1, ac2, f;
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
    if (!startmove()) return 0;
  }
#if defined(ESP8266)
  feedthedog();
#endif


#ifdef ISPC
  if (1) {
#else
  cm = micros();
  if (cm - nextmicros >= dl) {
#endif

    // update delay not every step to make the motor move faster
    if (m->status & 8) {
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

    float cf = float(timescale);
    cf /= dl * stepmmx[m->fastaxis];
    //c = cf;
    //c=COLOR(c,c,c);

    //pset (tick*tickscale+10,400-f*fscale),c
#define realpos1(i) (m->xs[i] / stepmmx[i])
    if (tick * tickscale / timescale > 600)tick = 0;
    //putpixel (tick * tickscale / timescale + 30, 450 -  fscale * cf, c - 8);
    int gx = tick * tickscale / timescale + 10;
    int gy = 400 -  fscale * f / TMSCALE / stepmmx[m->fastaxis];
    putpixel (gx, gy, c);
    setcolor(0);
    line(gx + 1, 400, gx + 1, 400 - 100 * fscale);

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

#ifdef output_enable
    // write real speed too
    if (mctr % 60 == 0) {
      //float cf = float(timescale);
      //cf /= dl * stepmmx[m->fastaxis];
      //zprintf(PSTR("F:%f D:%d\n"), ff(f / stepmmx[m->fastaxis] / TMSCALE), dl);
    }
#endif
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
  if (m || (head == tail) ) return 0; // if empty then exit
  int32_t t = nextbuff(tail);
  int ix;
  m = &move[t];
  // if ((m->status & 3) == 1) {

  nextmicros = micros();
  // do backlash correction
  prepareramp(t);
  f = m->fs;// m->fs*= stepmmx[m->fastaxis]; in planner
  dl = timescaleLARGE / f;
  motor_0_DIR(m->sx[0]);
  motor_1_DIR(m->sx[1]);
  motor_2_DIR(m->sx[2]);
  motor_3_DIR(m->sx[3]);
  mcx[0] = mcx[1] = mcx[2] = mcx[3] = (m->totalstep / 2);
  //if (m->fe==0)zprintf(PSTR("???\n"));
  tail = t;
  m->status &= ~3;
  m->status |= 2;
#ifdef output_enable
  zprintf(PSTR("Start tail:%d head:%d\n"), fi(tail), fi(head));
  zprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), m->rampup, m->rampdown, m->totalstep);
  zprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs / stepmmx[m->fastaxis] / TMSCALE), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
  //xprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
  xprintf(PSTR("sx %d %d %d \n"), fi(m->sx[0]), fi(m->sx[1]), fi(m->sx[2]));
  xprintf(PSTR("Status:%d \n"), fi(m->status));
#endif

  ac1 = m->ac1;
  ac2 = m->ac2;
  //nextdly = 0;
  mctr = 0;
  return 1;

}

/*
    =================================================================================================================================================
    DOCHECKENDSTOP
    =================================================================================================================================================
*/
#ifdef INVERTENDSTOP
#define ENDCHECK !
#define ENDPIN INPUT
#else
#define ENDCHECK
#define ENDPIN INPUT_PULLUP
#endif

void docheckendstop()
{
  // AVR specific code here
  // read end stop
#ifdef xmin_pin
  endstopstatus[0] = xmin_pin;
#elif defined(xmax_pin)
  endstopstatus[0] = xmax_pin;
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



#ifndef ISPC
  for (int32_t e = 0; e < 3; e++) {
    if (ENDCHECK endstopstatus[e]) {
      endstopstatus[e] = -digitalRead(endstopstatus[e]);
    }
  }
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
      xx[e] =  - ENDSTOP_MOVE;\
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
    xx[e] =  - ENDSTOP_MOVE;\
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

#ifndef ISPC
#ifdef xmin_pin
  pinMode(xmin_pin, ENDPIN);
#endif
#ifdef xmax_pin
  pinMode(xmax_pin, ENDPIN);
#endif

#ifdef ymin_pin
  pinMode(ymin_pin, ENDPIN);
#endif
#ifdef ymax_pin
  pinMode(ymax_pin, ENDPIN);
#endif

#ifdef zmin_pin
  pinMode(zmin_pin, ENDPIN);
#endif
#ifdef zmax_pin
  pinMode(zmax_pin, ENDPIN);
#endif

#endif
}



