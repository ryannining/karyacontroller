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



int homingspeed;
float homeoffset[4] ;
int jerk[4] ;
int accel[4];
int mvaccel[4];
int maxf[4];
float stepmmx[4];
int8_t checkendstop;
int8_t endstopstatus[3];
int32_t head, tail;
tmove move[NUMBUFFER];
float cx1, cy1, cz1, ce01, lf;
float ax_max[3];

void reset_motion(){
    homingspeed=HOMINGSPEED;
    homeoffset[0]=XOFFSET;
    homeoffset[1]=YOFFSET;
    homeoffset[2]=ZOFFSET;
    homeoffset[3]=EOFFSET;

    maxf[0]=XMAXFEEDRATE;
    maxf[1]=YMAXFEEDRATE;
    maxf[2]=ZMAXFEEDRATE;
    maxf[3]=E0MAXFEEDRATE;
    

    jerk[0]=XJERK;
    jerk[1]=YJERK;
    jerk[2]=ZJERK;
    jerk[3]=E0JERK;

    accel[0]=XACCELL;
    accel[1]=YACCELL;
    accel[2]=ZACCELL;
    accel[3]=E0ACCELL;

    mvaccel[0]=XMOVEACCELL;
    mvaccel[1]=YMOVEACCELL;
    mvaccel[2]=ZMOVEACCELL;
    mvaccel[3]=E0ACCELL;
    
    stepmmx[0]=XSTEPPERMM;
    stepmmx[1]=YSTEPPERMM;
    stepmmx[2]=ZSTEPPERMM;
    stepmmx[3]=E0STEPPERMM;
    
    ax_max[0]=XMAX;
    ax_max[1]=YMAX;
    ax_max[2]=ZMAX;
    
    checkendstop=0;
    endstopstatus[0]=0;
    endstopstatus[1]=0;
    endstopstatus[2]=0;
    head=tail=0;
    cx1=0;
    cy1=0;
    cz1=0;
    ce01=0;
    tick = 0;
    
}

int32_t mcx[NUMAXIS];

class tmotor {
  public:
    int32_t enable;
    int axis, pinenable, pinstep, pindir;
    void stepping();
    void init(int ax);
    void onoff(int e);
    void setdir(int dx);

};
void tmotor::onoff(int e) {
  enable = e ? 0 : 1;
#if defined(__AVR__) || defined(ESP8266)
  digitalWrite(pinenable, enable);
#endif
}
void tmotor::init(int ax) {
  axis = ax;
  enable = 1;
  pinenable=0;
  switch (ax) {
    case 0:
      #ifdef xstep
      pinenable = xenable;
      pinstep = xstep;
      pindir = xdirection;
      #endif
      break;
    case 1:
      #ifdef ystep
      pinenable = yenable;
      pinstep = ystep;
      pindir = ydirection;
      #endif
      break;
    case 2:
      #ifdef zstep
      pinenable = zenable;
      pinstep = zstep;
      pindir = zdirection;
      #endif
      break;
    case  3:
      #ifdef e0step
      pinenable = e0enable;
      pinstep = e0step;
      pindir = e0direction;
      #endif
      break;
    default:;
  }
#if defined(__AVR__) || defined(ESP8266)  
  if (pinenable)digitalWrite(pinenable, enable);
#endif
}
void tmotor::setdir(int dx) {
#if defined(__AVR__) || defined(ESP8266)
  digitalWrite(pindir, dx > 0 ? 1 : 0);
#endif
}
void tmotor::stepping()
{
#if defined(__AVR__) ||  defined(ESP8266)
  //xprintf("Stepping %d %d",(uint32_t)axis,(uint32_t)dx);
  digitalWrite(pinstep, 1);
  //delayMicroseconds(2);
  digitalWrite(pinstep, 0);
#else
#endif
}

tmotor mymotor[NUMAXIS];

int mb_ctr;
int32_t bufflen() {
  mb_ctr = head >= tail ? head - tail : (NUMBUFFER + head) - tail; // 5+0 - 4
  return mb_ctr;
}

/*
  safespeed
  =========
  Bertujuan mencari kecepatan aman dari sebuah gerakan, misalnya gerakan dg speed 100, dan max speed 50, maka perlu diskala 0.5

*/
void safespeed(tmove *m) {
  int32_t i;

  float  scale = 1;

  for (i = 0; i < NUMAXIS; i++) {
    if (m->dx[i] > 0) {
      m->fx[i] = m->fn * m->dx[i] / m->totalstep;
      //print32_t .fx(i)

      float scale2 = maxf[i] / m->fx[i];
      if (scale2 < scale) scale = scale2;
    }
  }
  // update all speed

  m->fn = m->fn * scale;
  for (i = 0; i < NUMAXIS; i++) {
    m->fx[i] = m->fx[i] * scale * m->sx[i];
    //xprintf("F%d:%f\n",i,ff(m->fx[i]));
  }
  //print32_t "w",.fx(1),.fx(2),.fx(3)
}
/*
  prepareramp
  ===========
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
  if (m->g0)  ac = mvaccel[m->fastaxis];else ac = accel[m->fastaxis];
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
  m->ac1 /= timescale;
  m->ac2 /= timescale;
}

/*
  planner
  =======
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
  addmove
  =======
  Rutin menambahkan sebuah vektor ke dalam buffer gerakan
*/
float x1[NUMAXIS], x2[NUMAXIS];
void addmove(float cf, float cx2, float cy2 , float cz2, float ce02,int8_t g0 )
{
  //cf=100;
#ifdef output_enable
  xprintf(PSTR("Tail:%d Head:%d \n"), tail, head);
  xprintf(PSTR("From X:%f Y:%f Z:%f\n"), ff(cx1), ff(cy1), ff(cz1));
  xprintf(PSTR("to F:%f X:%f Y:%f Z:%f\n"), ff(cf), ff(cx2), ff(cy2), ff(cz2));
#endif
  //zprintf(PSTR("-\n"));
  needbuffer();
  tmove *am;
  am = &move[nextbuff(head)];
  am->g0=g0;
  am->col = 2 + (head % 2) * 8;
  x1[0] = cx1 * stepmmx[0];
  x1[1] = cy1 * stepmmx[1];
  x1[2] = cz1 * stepmmx[2];
  x1[3] = ce01 * stepmmx[3];

  x2[0] = cx2 * stepmmx[0];
  x2[1] = cy2 * stepmmx[1];
  x2[2] = cz2 * stepmmx[2];
  x2[3] = ce02 * stepmmx[3];
  am->fn = cf;
  am->fe = 0;
  am->fs = 0;
  am->planstatus = 0; //0: not optimized 1:fixed
  am->status = 1; // 0: finish 1:ready 2:running
  //calculate delta
  int32_t ix;
  for (ix = 0; ix < NUMAXIS; ix++) {
    int32_t delta = (x2[ix] - x1[ix]);
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
  cx1 = cx2;
  cy1 = cy2;
  cz1 = cz2;
  ce01 = ce02;
  am->status = 1; // 0: finish 1:ready 2:running
}


tmove *m = 0;
float tick, tickscale, fscale;
int32_t px[4] = {0, 0, 0, 0 };
float f, dl;
int32_t mctr;
uint32_t nextmicros;
uint32_t nextmotoroff;


/*
    Inti dari motion loop






*/

void motionloop() {
  uint32_t cm, ix;
  feedthedog();
#if defined(__AVR__) || defined(ESP8266)
  //delayMicroseconds(1);
#endif
#if defined(__AVR__) || defined(ESP8266)
  cm = micros();
  temp_loop(cm);
  if ((nextmotoroff < cm) && (cm - nextmotoroff < 400000)) {
    //xprintf(PSTR("Motor off\n"));
    nextmotoroff = cm + motortimeout;
    for (ix = 0; ix < NUMAXIS; ix++) {
      mymotor[ix].onoff(0);
    }
  }
#endif
  if (!m ) {
    // start new move if available , if not exit
    if (!startmove()) return;
  }
  if (m->status == 2) {
    // debug the micros
#if defined(__AVR__) || defined(ESP8266)
    cm = micros();
    if ((nextmicros < cm) && (cm - nextmicros < 100000)) {
#else
    if (1) {
#endif
      if (m->totalstep) {
        if (f > 0)
          dl = timescale / (f * stepmmx[m->fastaxis]);
        else
          dl = timescale / stepmmx[m->fastaxis];

        tick = tick + dl;//1.0/timescale;
        int32_t c = m->col;
        //xprintf("Speed: %f\n",f);
        //if (mctr % 60==0)xprintf(PSTR("%d"),nextmicros);

        nextmotoroff = cm + motortimeout;
#if defined(__AVR__) || defined(ESP8266)
        // ESP8266 specific code here
        nextmicros = cm + dl;
#else
        nextmicros = cm + dl;

        //pset (tick*tickscale+10,400-f*fscale),c
        putpixel (tick * tickscale / timescale + 10, 400 - f * fscale, c);
        putpixel (px[0] / stepmmx[0] + 50, px[1] / stepmmx[1] + 40, c);
        //pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
#endif
        //if (mctr % 60==0)xprintf("F:%f Dly-%fus\n", ff(f),ff(dl));
        // bresenham work on motor step
        // v2, much simpler
        for (ix = 0; ix < NUMAXIS; ix++) {
          if (m->sx[ix]) {
            mcx[ix] -= m->dx[ix];
            if (mcx[ix] < 0) {
              px[ix] += m->sx[ix];
              mcx[ix] += m->totalstep;
              mymotor[ix].stepping();
            }
          }
        }
        /*
                for (ix = 0; ix < NUMAXIS; ix++) {
                  if (m->sx[ix]) {
                    m->cx[ix] = m->cx[ix] + m->dx[ix];
                    if ((m->sx[ix]) && (m->cx[ix] >= m->totalstep)) {
                      x[ix] = x[ix] + m->sx[ix];
                      mymotor[ix].stepping();
                      m->cx[ix] = m->cx[ix] - m->totalstep;
                    }
                  }
                }

        */
        // next speed
        if (mctr < m->rampup) {
          //if (m->ac1>0 and f<.fn) or ( .ac1<0 and f>.fn) then
          f = f + m->ac1 * dl;
          //if (f < 1)f = 1;
          //if ((m->ac1>0) && (f>m->fn))f=m->fn;
          //if ((m->ac1<0) && (f<m->fn))f=m->fn;
          //xprintf("Dl:%f\n",ff(dl*timescale));
        }
        else if (mctr > m->totalstep - m->rampdown )
        {
          //if (.ac2>0 and f<=.fe) or ( .ac2<0 and f>=.fe) then
          f = f + m->ac2 * dl;
          //if (f < 1)f = 1;
          //if ((m->ac2>0) && (f>m->fn))f=m->fn;
          //if ((m->ac2<0) && (f<m->fn))f=m->fn;
          //if (f>m->fn)f=m->fn;

        }
        else {
          f = m->fn;

        }
        //if (m->bpos==8)
        //    if (mctr % 60==0)xprintf(PSTR("F:%f\n"),ff(f));
        mctr++;

        // delay (dl*F_CPU)
        // next timer
        // if finish call
      }
      if (checkendstop) {
        docheckendstop();
        for (int32_t e = 0; e < 3; e++)
        {
          if (endstopstatus[e] < 0) {
            m->status = 0;
            m = 0;
            checkendstop = 0;
            //xprintf(PSTR("here\n"));
            cx1 = px[0] / stepmmx[0];
            cy1 = px[1] / stepmmx[1];
            cz1 = px[2] / stepmmx[2];
            ce01 = px[3] / stepmmx[3];
            return;
          }
        }
      }
      if (mctr >= m->totalstep) {
        //xprintf(PSTR("Finish:%d\n"),mctr);
        m->status = 0;
        m = 0;
      }
      //

    }
  }
}

void docheckendstop()
{
#if defined(__AVR__)
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


  for (int32_t e = 0; e < NUMAXIS; e++) {
    if (endstopstatus[e]) {
      endstopstatus[e] = -digitalRead(endstopstatus[e]);
    }
  }
#elif defined(ESP8266)
  // ESP8266 specific code here
  // read end stop

#else
  if (!m) return;
  // simulate endstop if x y z is 0
  for (int32_t e = 0; e < NUMAXIS; e++) {
    if (x1[e] < 0) endstopstatus[e] = 1; else  endstopstatus[e] = 0;
  }
#endif
}
void homing(float x, float y, float z, float e0)
{
  //addmove(homingspeed, 0, 0, 0,ce01);
  //waitbufferempty();
  m = 0;
  head = tail;
  int32_t tx[3];
  tx[0] = cx1;
  tx[1] = cy1;
  tx[2] = cz1;
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
  docheckendstop();
  float xx[3] = {cx1, cy1, cz1};
  //xprintf(PSTR("ENDSTOP %d %d %d\n"), (int32_t)endstopstatus[0], (int32_t)endstopstatus[1], (int32_t)endstopstatus[2]);
  //xprintf(PSTR("Position %f %f %f \n"), ff(cx1), ff(cy1), ff(cz1));
  for (int32_t e = 0; e < 3; e++) {
    // move away from endstop
    if (tx[e]) {
      xx[e] = px[e] / stepmmx[e] - tx[e] / 100;
      checkendstop = 0;
      addmove(homingspeed/10, xx[0], xx[1], xx[2], ce01);
      waitbufferempty();
      // check endstop again slowly
      xx[e] = tx[e];
      checkendstop = 1;
      addmove(homingspeed / 20, xx[0], xx[1], xx[2], ce01);
      waitbufferempty();
      // move away again
      xx[e] = px[e] / stepmmx[e] - tx[e] / 500;
      checkendstop = 0;
      addmove(homingspeed / 20, xx[0], xx[1], xx[2], ce01);
      waitbufferempty();
    }
  }
  checkendstop = ce01 = 0;

#ifdef xmax_pin
  cx1 = ax_max[0];
  px[0] = cx1 * stepmmx[0];
#else
  cx1 = px[0] = 0;
#endif

#ifdef ymax_pin
  cy1 = ax_max[1];
  px[1] = cy1 * stepmmx[1];
#else
  cy1 = px[1] = 0;
#endif

#ifdef zmax_pin
  cz1 = ax_max[2];
  px[2] = cz1 * stepmmx[2];
#else
  cz1 = px[2] = 0;
#endif
  ce01 = 0;
  px[3]= 0; 
  //xprintf(PSTR("Home to:X:%f Y:%f Z:%f\n"),  ff(cx1), ff(cy1), ff(cz1));

}
/*
  startmove
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
      for (ix = 0; ix < NUMAXIS; ix++) {
        mymotor[ix].onoff(1);
        mymotor[ix].setdir(m->sx[ix]);
        mcx[ix] = (m->totalstep - 1) / 2;
      }
      prepareramp(t);
#ifdef output_enable
      xprintf(PSTR("Start buff:%d\n"), tail);
      xprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), m->rampup, m->rampdown, m->totalstep);
      xprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));
      xprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
      xprintf(PSTR("sx %d %d %d \n"), fi(m->sx[0]), fi(m->sx[1]), fi(m->sx[2]));
#endif
      tail = t;
      m->status = 2;
      f = m->fs;
      if (f == 0)  f = 1;
      mctr = 0;
      return 1;
    }

  }
  return 0;
}

/*
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
  needbuffer
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
  inisialisasi awal, wajib
*/
void initmotion() {

  reset_motion();  
  tickscale = 120;
  fscale = 2;
  int32_t i;
  for (i = 0; i < NUMAXIS; i++) {
    mymotor[i].init(i);
  }
#if defined(__AVR__) || defined(ESP8266)
#ifdef xenable
  pinMode(xenable, OUTPUT);
#endif
#ifdef xdirection
  pinMode(xdirection, OUTPUT);
#endif
#ifdef xstep
  pinMode(xstep, OUTPUT);
#endif
#ifdef yenable
  pinMode(yenable, OUTPUT);
#endif
#ifdef ydirection
  pinMode(ydirection, OUTPUT);
#endif
#ifdef ystep
  pinMode(ystep, OUTPUT);
#endif
#ifdef zenable
  pinMode(zenable, OUTPUT);
#endif
#ifdef zdirection
  pinMode(zdirection, OUTPUT);
#endif
#ifdef zstep
  pinMode(zstep, OUTPUT);
#endif
#ifdef e0enable
  pinMode(e0enable, OUTPUT);
#endif
#ifdef e0direction
  pinMode(e0direction, OUTPUT);
#endif
#ifdef e0step
  pinMode(e0step, OUTPUT);
#endif

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



