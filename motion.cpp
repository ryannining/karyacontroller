#include "motion.h"
#include "common.h"
#include "timer.h"
#if defined(__AVR__)



#include<arduino.h>
#define xenable 2
#define xdirection 6
#define xstep 4

#define yenable 7
#define ydirection 9
#define ystep 8

#define zenable 10
#define zdirection 5
#define zstep 27


#elif defined(ESP8266)
#include<arduino.h>
#define xenable D1
#define xdirection D2
#define xstep D3
#define yenable D1
#define ydirection D2
#define ystep D3
#define zenable D1
#define zdirection D2
#define zstep D3


#else
#include <graphics.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <conio.h>

#define xenable 0
#define xdirection 1
#define xstep 2
#define yenable 7
#define ydirection 9
#define ystep 8
#define zenable 10
#define zdirection 5
#define zstep 27
#endif


#define motortimeout 10000000 // 30 seconds

// need to change depends on CPU
#define F_CPU 16000000UL
#define  US  * (F_CPU / 1000000)
#define MS  * (F_CPU / 1000)

float homingspeed = 30;
float homeoffset[numaxis] = {0, 0, 0};
float jerk[numaxis] = {40, 40, 40};
float accel[numaxis] = {1000, 1000, 100};
float maxf[numaxis] = {150, 150, 150};
float stepmmx[numaxis] = {112, 112, 405};
tmove move[numbuffer];
float cx1, cy1, cz1, lf;

uint8_t checkendstop = 0;
uint8_t endstopstatus[numaxis] = {0, 0, 0};

int32_t head, tail = 0;

class tmotor {
  public:
    int32_t enable;
    int axis,pinenable,pinstep,pindir;
    void stepping(int32_t dx);
    void init(int ax);
    void onoff(int e);

};
void tmotor::onoff(int e){
  enable=e?0:1;
  digitalWrite(pinenable, enable);
}
void tmotor::init(int ax){
  axis=ax;
  enable=1;
  if (ax==0){
    pinenable=xenable;
    pinstep=xstep;
    pindir=xdirection;
  } else if(ax==1){
    pinenable=yenable;
    pinstep=ystep;
    pindir=ydirection;
    
  } else if(ax==3){
    pinenable=zenable;
    pinstep=zstep;
    pindir=zdirection;
    
  }
#if defined(__AVR__) || defined(ESP8266) 
  digitalWrite(pinenable, enable);
#endif
}
void tmotor::stepping(int32_t dx)
{
#if defined(__AVR__) ||  defined(ESP8266) 
  //xprintf("Stepping %d %d",(uint32_t)axis,(uint32_t)dx);
  digitalWrite(pindir, dx>0?1:0);
  digitalWrite(pinstep, 1);
  //delayMicroseconds(2);
  digitalWrite(pinstep, 0);
#else
#endif
}

tmotor mymotor[numaxis];

int mb_ctr;
int32_t bufflen() {
    mb_ctr=head>=tail?head-tail:(numbuffer+head)-tail; // 5+0 - 4 
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

  for (i = 0; i < numaxis; i++) {
    if (m->dx[i] > 0) {
      m->fx[i] = m->fn * m->dx[i] / m->totalstep;
      //print32_t .fx(i)

      float scale2 = maxf[i] / m->fx[i];
      if (scale2 < scale) scale = scale2;
    }
  }
  // update all speed

  m->fn = m->fn * scale;
  for (i = 0; i < numaxis; i++) {
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
  ac = accel[m->fastaxis];
  m->ac1 = ACCELL(m->fs, m->fn, ac);
  m->ac2 = ACCELL(m->fn, m->fe, ac);
  m->rampup = ramplen(m->fs, m->fn, m->ac1, stepmm);
  m->rampdown = ramplen(m->fe, m->fn, m->ac2, stepmm);

#define preprampdebug

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
    } else
    if (m->rampdown > m->totalstep) {
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
  m->ac1/=timescale;
  m->ac2/=timescale;
}

/*
  planner
  =======
  dipanggil oleh addmove untuk mulai menghitung dan merencakanan gerakan terbaik
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
  if (prev->status==2)return;
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
  for (i = 0; i < numaxis; i++) {
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
float x1[numaxis], x2[numaxis];
void addmove(float cf, float cx2, float cy2 , float cz2 )
{
  //cf=100;
  xprintf(PSTR("Tail:%d Head:%d "), tail, head);
  xprintf(PSTR("F:%f X:%f Y:%f Z:%f\n"), ff(cf),ff(cx2),ff(cy2),ff(cz2));
  needbuffer();
  tmove *am;
  am = &move[nextbuff(head)];

  am->col = 2 + (head % 2) * 8;
  x1[0] = cx1 * stepmmx[0];
  x1[1] = cy1 * stepmmx[1];
  x1[2] = cz1 * stepmmx[2];
  x2[0] = cx2 * stepmmx[0];
  x2[1] = cy2 * stepmmx[1];
  x2[2] = cz2 * stepmmx[2];
  am->fn = cf;
  am->fe = 0;
  am->fs = 0;
  am->planstatus = 0; //0: not optimized 1:fixed
  am->status = 1; // 0: finish 1:ready 2:running
  //calculate delta
  int32_t ix;
  for (ix = 0; ix < numaxis; ix++) {
    int32_t delta = (x2[ix] - x1[ix]);
    am->dx[ix] = fabs(delta);

    if (delta > 0) am->sx[ix] = 1;
    else if (delta < 0) am->sx[ix] = -1;
    else am->sx[ix] = 0;
    am->cx[ix] = 0;
    //xprintf("Dx%d:%d\n",ix,m->dx[ix]);
    //xprintf("Sx%d:%d\n",ix,m->sx[ix]);
    //xprintf("Cx%d:%d\n",ix,m->cx[ix]);
  }
  for (ix = 0; ix < numaxis; ix++) {
    if ((am->dx[ix] >= am->dx[0]) && (am->dx[ix] >= am->dx[1]) && (am->dx[ix] >= am->dx[2]))  {
      am->fastaxis = ix;
    }
  }
  am->totalstep = am->dx[am->fastaxis];
  xprintf(PSTR("Totalstep AX%d %d\n"),(int32_t)am->fastaxis,(int32_t)am->totalstep);

  // } buffer please

  // back planner
  head = nextbuff(head);
  am->bpos = head;
  planner(head);
  cx1 = cx2;
  cy1 = cy2;
  cz1 = cz2;
  am->status = 1; // 0: finish 1:ready 2:running
}


tmove *m = 0;
float tick, tickscale, fscale;
float x[numaxis] = {0, 0, 0 };
float f, dl;
int32_t mctr;
uint32_t nextmicros;
uint32_t nextmotoroff;

int32_t motionrunning = 0;

/*
    Inti dari motion loop






*/

void motionloop() {
  uint32_t cm,ix;
  feedthedog();
#if defined(__AVR__) || defined(ESP8266)
  delayMicroseconds(1);
#endif
  #if defined(__AVR__) || defined(ESP8266)
  cm=micros();
  if ((nextmotoroff < cm) && (cm-nextmotoroff<400000)) {
    printf(PSTR("Motor off\n"));
    nextmotoroff =cm+motortimeout;
    for(ix=0;ix<numaxis;ix++){
      mymotor[ix].onoff(0);
    }
  }
  #endif
  if (!m ) {
    // start new move if available , if not exit
    motionrunning = 0;
    if (!startmove()) return;
  }
  if (m->status == 2) {
    // debug the micros
    #if defined(__AVR__) || defined(ESP8266)
    cm=micros();
    if ((nextmicros < cm) && (cm-nextmicros<100000)) {
    #else
    if(1){
    #endif
    if (m->totalstep) {
        motionrunning = 1;
        if (f > 0)
          dl = timescale / (f * stepmmx[m->fastaxis]);
        else
          dl = timescale / stepmmx[m->fastaxis];

        tick = tick + dl;//1.0/timescale;
        int32_t c = m->col;
        //xprintf("Speed: %f\n",f);
        //if (mctr % 60==0)xprintf(PSTR("%d"),nextmicros);
        
        nextmotoroff=cm+motortimeout;
#if defined(__AVR__) || defined(ESP8266)
        // ESP8266 specific code here
        nextmicros = cm + dl;
#else
        nextmicros = cm + dl;

        //pset (tick*tickscale+10,400-f*fscale),c
        putpixel (tick * tickscale/timescale + 10, 400 - f * fscale, c);
        putpixel (x[0] / stepmmx[0] + 50, x[1] / stepmmx[1] + 40, c);
        //pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
#endif
        //if (mctr % 60==0)xprintf("F:%f Dly-%fus\n", ff(f),ff(dl));
        // bresenham work on motor step
        for (ix = 0; ix < numaxis; ix++) {
          if (m->sx[ix]) {
            m->cx[ix] = m->cx[ix] + m->dx[ix];
            if ((m->sx[ix]) && (m->cx[ix] >= m->totalstep)) {
              x[ix] = x[ix] + m->sx[ix];
              mymotor[ix].stepping(m->sx[ix]);
              m->cx[ix] = m->cx[ix] - m->totalstep;
            }
          }
        }
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
            f=m->fn;

        }
        //if (m->bpos==8)
        //    if (mctr % 60==0)xprintf(PSTR("F:%f\n"),ff(f));
        mctr++;

        // delay (dl*F_CPU)
        // next timer
        // if finish call
      }
      if (checkendstop) {
        for (int32_t e = 0; e < numaxis; e++)
        {
          if (endstopstatus[e]) {
            m->status = 0;
            motionrunning = 0;
            m = 0;
            return;
          }
        }
      }
      if (mctr >= m->totalstep) {
        //xprintf(PSTR("Finish:%d\n"),mctr);
        m->status = 0;
        motionrunning = 0;
        m=0;
      }
      //

    }
  }
}

int32_t docheckendstop()
{
#if defined(__AVR__)
  // AVR specific code here
  // read end stop
#elif defined(ESP8266)
  // ESP8266 specific code here
  // read end stop

#else
  if (!m) return 0;
  // simulate endstop if x y z is 0
  for (int32_t e = 0; e < numaxis; e++) {
    if (x1[e] < 0) endstopstatus[e] = 1; else  endstopstatus[e] = 0;
  }
#endif
}
void homing(float x, float y, float z)
{
  m = 0;
  head = tail;
  checkendstop = 1;
  addmove(homingspeed, -100, -100, -100);
  startmove();
  waitbufferempty();
  // now slow down and check endstop once again
  cx1 = cy1 = cz1 = 0;
  float xx[numaxis] = {0, 0, 0};
  for (int32_t e = 0; e < numaxis; e++) {
    // move away from endstop
    xx[e] = 10;
    checkendstop = 0;
    addmove(homingspeed, xx[0], xx[1], xx[2]);
    waitbufferempty();
    // check endstop again slowly
    xx[e] = 0;
    checkendstop = 1;
    addmove(homingspeed / 5, xx[0], xx[1], xx[2]);
    waitbufferempty();
    // move away again
    xx[e] = 2;
    checkendstop = 0;
    addmove(homingspeed / 5, xx[0], xx[1], xx[2]);
    waitbufferempty();
  }
  checkendstop = 0;
  cx1 = x;
  cy1 = y;
  cz1 = z;
}
/*
  startmove
  Mulai menjalankan 1 unit di buffer gerakan terakhir
*/
int32_t startmove()
{
  if (motionrunning) exit;
  if (head != tail) { // if empty then exit
    int32_t t = nextbuff(tail);
    int ix;
    m = &move[t];
    if (m->status == 1) {
        xprintf(PSTR("Start buff:%d\n"),tail);
      if (0) {  
        
        xprintf(PSTR("RU:%d Rd:%d Ts:%d\n"), m->rampup, m->rampdown,m->totalstep);
        xprintf(PSTR("FS:%f AC:%f FN:%f AC:%f FE:%f\n"), ff(m->fs), ff(m->ac1), ff(m->fn), ff(m->ac2), ff(m->fe));

      }
      for(ix=0;ix<numaxis;ix++){
        mymotor[ix].onoff(1);
      }
      prepareramp(t);
      tail = t;
      m->status = 2;
      f = m->fs;
      if (f == 0)  f = 1;
      mctr = 0;
      nextmicros = micros() + 100;
      motionrunning = 1;
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
while ((head!=tail) || m) {
    motionloop();
  }
}
/*
  needbuffer
  loop sampai da buffer yang bebas
*/
void needbuffer()
{
  xprintf(PSTR("buffer %d / %d \n"), tail, head);
  if (nextbuff(head)==tail) {
    xprintf(PSTR("Wait\n"));
      while (m) {
        motionloop();
      }
      startmove();
      int lt = tail;
      while (m) {
        motionloop();
      }
      m=0;
      xprintf(PSTR("Done\n"));
    }
}
/*
  inisialisasi awal, wajib
*/
void initmotion() {


  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  lf = 0;
  tick = 0;
  tickscale = 120;
  fscale = 2;
  head = 0;
  tail = 0;
  int32_t i;
  for (i = 0; i < numaxis; i++) {
    mymotor[i].init(i);
  }
#if defined(__AVR__) || defined(ESP8266)
  pinMode(xenable, OUTPUT);
  pinMode(xdirection, OUTPUT);
  pinMode(xstep, OUTPUT);
  pinMode(yenable, OUTPUT);
  pinMode(ydirection, OUTPUT);
  pinMode(ystep, OUTPUT);
  pinMode(zenable, OUTPUT);
  pinMode(zdirection, OUTPUT);
  pinMode(zstep, OUTPUT);
#endif
}



