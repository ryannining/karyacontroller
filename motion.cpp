
#include "motion.h"
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



uint8_t homingspeed;
uint8_t homeoffset[4] ;
uint8_t jerk[4] ;
uint8_t xback[4];
uint8_t head, tail;
uint8_t maxf[4];
uint8_t f_multiplier, e_multiplier;
int8_t checkendstop;
int8_t endstopstatus[3];
int accel[4];
int i;
int mvaccel[4];
uint16_t ax_max[3];
uint32_t dl, totalstep;
float stepmmx[4];
float cx1, cy1, cz1, ce01;
tmove move[NUMBUFFER];

#define ramplen(oo,v0,v1,a ,stepmm) t = (v1 - v0) / a;oo=fabs((v0 * t + 0.5 * a * t * t) * stepmm);

#define speedat(v0,a,s,stp) sqrt(a * 2 * s / stp + v0 * v0)
#define accelat(v0,v1,s) (v1 * v1 - v0 * v0) / (2 * s)
#define ACCELL(v0,v1,a) v0<v1?a:-a

/*
    =================================================================================================================================================
    RESET_MOTION
    =================================================================================================================================================
  Reset all variable
*/
// keep last direction to enable backlash if needed
int8_t lsx[4] = {0, 0, 0, 0};

void reset_motion() {
  e_multiplier=f_multiplier=255;
  
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

  ax_max[0] = XMAX*10;
  ax_max[1] = YMAX*10;
  ax_max[2] = ZMAX*10;

  checkendstop = 0;
  endstopstatus[0] = 0;
  endstopstatus[1] = 0;
  endstopstatus[2] = 0;
  head = tail = 0;
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  ce01 = 0;
#ifdef ISPC
  tick = 0;
#endif
  lsx[0] = 0;
  lsx[1] = 0;
  lsx[2] = 0;
  lsx[3] = 0;
  xback[0] = MOTOR_X_BACKLASH;
  xback[1] = MOTOR_Y_BACKLASH;
  xback[2] = MOTOR_Z_BACKLASH;
  xback[3] = MOTOR_E_BACKLASH;

  e_multiplier = 255;

}

int32_t mcx[NUMAXIS];
/*
    =================================================================================================================================================
    MOTOR CLASS
    =================================================================================================================================================
*/

#define CORELOOP
#define FASTAXIS(n) n->status >> 4
//if(m)coreloop();
#include "motors.h"

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

  float  scale = 1;
  #define xtotalstep  m->dx[FASTAXIS(m)]
  for (i = 0; i < NUMAXIS; i++) {
    if (m->dx[i] > 0) {
      m->fx[i] = m->fn * m->dx[i] / xtotalstep;
      //print32_t .fx(i)

      //xprintf(PSTR("MF:%f F:%f\n"),ff((float)maxf[i]),ff((float)m->fx[i]));
      float scale2 = (float)maxf[i] / (float)m->fx[i];
      if (scale2 < scale) scale = scale2;
      CORELOOP

    }
  }
  // update all speed

  m->fn = m->fn * scale;
  //xprintf(PSTR("SCALE:%f\n"),ff(scale));
  for (i = 0; i < NUMAXIS; i++) {
    m->fx[i] = scale * m->fx[i] *  m->sx[i];
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

  float t, ac, ac1, ac2;
  int faxis = FASTAXIS(m);
  #define ytotalstep  m->dx[faxis]
  #define stepmm  stepmmx[faxis]
  if (m->status & 8)  ac = mvaccel[faxis]; else ac = accel[faxis];
  ac1 = ACCELL(m->fs, m->fn, ac);
  ac2 = ACCELL(m->fn, m->fe, ac);
  ramplen(m->rampup, m->fs, m->fn, ac1, stepmm);
  ramplen(m->rampdown, m->fe, m->fn, ac2, stepmm);

  //#define preprampdebug

#ifdef preprampdebug
  zprintf(PSTR("Bpos:%d\n"), (int32_t)bpos);
  zprintf(PSTR("----------1-----------\nRU:%d Rld:%d TS:%d\n "), m->rampup, m->rampdown, totalstep);
  zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
  // New generic algorithm, to calculate rampup and rampdown simpler, and faster
  // if rampup and ramp down overlap
  if (m->rampup + m->rampdown > ytotalstep) {
    int32_t r = ((m->rampdown + m->rampup) - ytotalstep) / 2;
    m->rampup -= r;
    m->rampdown -= r;
    // check if still larger that totalstep
    if (m->rampup > ytotalstep) {
      // adjust speed
      m->fn = speedat(m->fs, ac1, ytotalstep, stepmm);
      m->fe = m->fn;
      m->rampdown = 0;
      m->rampup = ytotalstep;
#ifdef preprampdebug
      zprintf(PSTR("========2========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
    } else if (m->rampdown > ytotalstep) {
      // adjust acceleration
      ac2 = accelat(m->fs, m->fe, ytotalstep) * stepmm;
      m->rampup = 0;
      m->rampdown = ytotalstep;
#ifdef preprampdebug
      zprintf(PSTR("========3========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
      zprintf(PSTR("FS:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->fn), ff(m->fe));
#endif
      CORELOOP
    } else {
      m->fn = speedat(m->fs, ac1, m->rampup, stepmm);
      CORELOOP

    }
  }


#ifdef preprampdebug
  zprintf(PSTR("========FINAL========\nRU:%d Rd:%d\n"), m->rampup, m->rampdown);
  zprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(ac1), ff(m->fn), ff(ac2), ff(m->fe));
#endif
  m->ac1 = ac1 * stepmm * TMSCALE * SUBMOTION / timescale ;
  m->ac2 = ac2 * stepmm * TMSCALE * SUBMOTION / timescale ;
  if (m->fs < 1) m->fs = 1;
  m->fs *= stepmm * TMSCALE * SUBMOTION;
  m->status |= 4;
  m->rampup = (ytotalstep - m->rampup) * SUBMOTION;
  m->rampdown *= SUBMOTION; // presubtract
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
  int32_t p;
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
  int cjerk = jerk[FASTAXIS(curr)];

  scale = 1;
  for (i = 0; i < NUMAXIS; i++) {
    //xprintf("DFX%d:%f %f\n",i,prev->fx[i],curr->fx[i]);
    scale2 = cjerk;
    scale2 /= abs(curr->fx[i] - prev->fx[i]);
    if (scale2 < scale)  scale = scale2;
    CORELOOP
  }
  //zprintf ("jerk scaling:%f\n",scale);

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
  if (head == tail) {
    //zprintf(PSTR("Empty !\n"));
  }
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
  x2[3] = x2[3]*e_multiplier/255;
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

  am->fn = cf * f_multiplier/255;
  am->fe = 0;
  am->fs = 0;
  //am->planstatus = 0; //0: not optimized 1:fixed
  //calculate delta
  int32_t dd;
  dd = 0;
  int faxis = 0;
  for (i = 0; i < NUMAXIS; i++) {
    int32_t delta = x2[i];
    am->dx[i] = abs(delta);
    am->sx[i] = 0;
    if (delta > 0) am->sx[i] = 1;
    if (delta < 0) am->sx[i] = -1;
    if (am->dx[i] > dd) {
      dd = am->dx[i];
      faxis = i;
    }
  }
  am->status |= faxis << 4;
  //am->totalstep = am->dx[faxis];
#ifdef output_enable
  xprintf(PSTR("Totalstep AX%d %d\n"), (int32_t)faxis, (int32_t)am->dx[faxis]);
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
#ifdef ISPC
float tick;
float tickscale, fscale, graphscale;
#endif
//int32_t px[4] = {0, 0, 0, 0 };
int32_t ac1, ac2, f;
int32_t mctr;
uint8_t fastaxis;
uint32_t nextmicros;
uint32_t nextdly;
uint32_t nextmotoroff;


/*
    =================================================================================================================================================
    MOTIONLOOP
    =================================================================================================================================================
*/
uint32_t cm, ocm, rampup, rampdown;
int coreloop() {
  if (mctr <= 0)return 0;
#ifdef ISPC
  if (1) {
#else
  cm = micros();
  if (cm - nextmicros >= dl) {
#endif
    nextmicros = cm;


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
    cf /= dl * stepmmx[fastaxis];
    //c = cf;
    //c=COLOR(c,c,c);

    //pset (tick*tickscale+10,400-f*fscale),c
#define realpos1(i) (m->xs[i] / stepmmx[i])
    if (tick * tickscale / timescale > 600)tick = 0;
    //putpixel (tick * tickscale / timescale + 30, 450 -  fscale * cf, c - 8);
    int gx = tick * tickscale / timescale + 10;
    int gy = 400 -  fscale * f / TMSCALE / stepmmx[fastaxis];
    putpixel (gx, gy, c);
    setcolor(0);
    //line(gx + 1, 400, gx + 1, 400 - 100 * fscale);

    putpixel (graphscale * realpos1(0) + 250, graphscale * realpos1(1) + 150, c);
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
    mcx[ix] += totalstep;\
    graphstep(ix) \
  }

#ifdef e0step
    bresenham(3)
    STEPDELAY
    motor_3_UNSTEP();
#endif
    bresenham(0);
    bresenham(1);
    bresenham(2);
    STEPDELAY
    motor_0_UNSTEP();
    motor_1_UNSTEP();
    motor_2_UNSTEP();
    // next speed
    if (mctr > m->rampup) {
      f += (ac1 * dl);
    }
    else if (mctr < m->rampdown ) // already subtracted from totalstep
    {
      f += (ac2 * dl);
    }
    mctr--;
    /* ================================================================================================================ */
    //if (m->bpos==8)

#ifdef output_enable
    // write real speed too
    if (mctr % 60 == 0) {
      //float cf = float(timescale);
      //cf /= dl * stepmmx[m->fastaxis];
      zprintf(PSTR("F:%f D:%d\n"), ff(f / stepmmx[fastaxis] / TMSCALE), dl);
    }
#endif
    if (checkendstop) {
      docheckendstop();
      if ((endstopstatus[0] < 0) || (endstopstatus[1] < 0) || (endstopstatus[2] < 0)) {
        m->status = 0;
        m = 0;
        checkendstop = 0;
        return 1;
      }
    }

  }
  return 1;
}
int motionloop() {

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
  if (mctr <= 0) {
    //xprintf(PSTR("Finish:%d\n"),mctr);
#ifdef AUTO_MOTOR_Z_OFF
    if (m->dx[2] == 0)motor_2_OFF();
#endif

    m->status = 0;
    m = 0;
    if (!startmove()) return 0;
  }
#if defined(ESP8266)
  feedthedog();
#endif


  return coreloop();

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
  nextmicros = micros();
  int32_t t = nextbuff(tail);

  m = &move[t];
  // if ((m->status & 3) == 1) {
  fastaxis = FASTAXIS(m);
  totalstep = mctr = m->dx[fastaxis] * SUBMOTION;
  //zprintf(PSTR("Fastaxis:%d\n"),fi(fastaxis));
  // do backlash correction
  prepareramp(t);
  f = m->fs;// m->fs*= stepmmx[m->fastaxis]; in planner
  dl = timescaleLARGE / f;
  motor_0_DIR(m->sx[0]);
  motor_1_DIR(m->sx[1]);
  motor_2_DIR(m->sx[2]);
  motor_3_DIR(m->sx[3]);
  dobacklash();
  mcx[0] = mcx[1] = mcx[2] = mcx[3] = (totalstep / 2);
  //if (m->fe==0)zprintf(PSTR("???\n"));
  tail = t;
  m->status &= ~3;
  m->status |= 2;
#ifdef output_enable
  zprintf(PSTR("Start tail:%d head:%d\n"), fi(tail), fi(head));
  zprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), m->rampup, m->rampdown, totalstep);
  zprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs / stepmmx[fastaxis] / TMSCALE), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
  //xprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
  xprintf(PSTR("sx %d %d %d \n"), fi(m->sx[0]), fi(m->sx[1]), fi(m->sx[2]));
  xprintf(PSTR("Status:%d \n"), fi(m->status));
#endif

  ac1 = m->ac1;
  ac2 = m->ac2;

  //nextdly = 0;
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
    if (endstopstatus[e]) {
      endstopstatus[e] = -(ENDCHECK digitalRead(endstopstatus[e]));
    }
  }
#else
  // PC
  if (!m) return;
  // simulate endstop if x y z is 0
  for (int32_t e = 0; e < NUMAXIS; e++) {
    if (x1[e] < 0) endstopstatus[e] < 0; else  endstopstatus[e] = 0;
  }
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
  waitbufferempty();
  int32_t tx[4];
  tx[0] =  tx[1] = tx[2] = 0;
#define mmax ENDSTOP_MOVE*100
#ifdef xmin_pin
  tx[0] = -mmax;
#elif defined(xmax_pin)
  tx[0] = mmax;
#endif

#ifdef ymin_pin
  tx[1] = -mmax;
#elif defined(ymax_pin)
  tx[1] = mmax;
#endif
#ifdef zmin_pin
  tx[2] = -mmax;
#elif defined(zmax_pin)
  tx[2] = mmax;
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
      xx[e] =  - tx[e]/100;\
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
    xx[e] =  - tx[e]/100;\
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
  cx1 = ax_max[0]*0.1;
#else
  cx1 = 0;
#endif

#ifdef ymax_pin
  cy1 = ax_max[1]*0.1;
#else
  cy1  = 0;
#endif

#ifdef zmax_pin
  cz1 = ax_max[2]*0.1;
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
  xprintf(PSTR("buffer %d / %d \n"), fi(tail), fi(head));
#endif
  if (nextbuff(head) == tail) {
#ifdef output_enable
    xprintf(PSTR("Wait\n"));
#endif
    //wait current move finish
    int t = tail;
    while (t == tail) {
      motionloop();
    }
    /*    // wait new move finish
        startmove();
        t = tail;
        while (t==tail) {
          motionloop();
        }
        m = 0;
    */
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



