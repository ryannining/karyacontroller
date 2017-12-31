#include "motion.h"
#include "common.h"
#include "timer.h"
#if defined(__AVR__) || defined(ESP8266)
    #include<arduino.h>
#else
    #include <graphics.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <stdint.h>
    #include <conio.h>
#endif


// need to change depends on CPU
#define F_CPU 16000000UL
#define  US  * (F_CPU / 1000000)
#define MS  * (F_CPU / 1000)

double homingspeed=30;
double homeoffset[numaxis]={0,0,0};

uint8_t checkendstop=0;
uint8_t endstopstatus[numaxis]={0,0,0};



class tmotor{
  public:
    int32_t enable;
    void stepping(int32_t dx);
    
    
};

void tmotor::stepping(int32_t dx)
{
  #if defined(__AVR__) 

      digitalWrite(2,1);
      if (dx<0) {
      digitalWrite(6,1);
      } else if (dx>0){
      digitalWrite(6,0);
      }
      digitalWrite(4,1);
      delayMicroseconds(5);
      digitalWrite(4,0);
  #elif defined(ESP8266)
  #endif
}

tmotor mymotor[numaxis];
double jerk[numaxis]={30,30,30}; 
double accel[numaxis]={10,10,50};
double maxf[numaxis]={100,100,100};
double stepmmx[numaxis]={45,45,45};
tmove move[numbuffer+1];
double cx1,cy1,cz1,lf;


int32_t bufflen(){
    
    int32_t a=head-tail;
    if (a<0) a=a+numbuffer;
    return a;
}

/*
  safespeed 
  =========
  Bertujuan mencari kecepatan aman dari sebuah gerakan, misalnya gerakan dg speed 100, dan max speed 50, maka perlu diskala 0.5
  
*/
void safespeed(tmove *m) {
        int32_t i;
    
        double  scale=1;
        
        for (i=0;i< numaxis;i++){
            if (m->dx[i]>0) {
                m->fx[i]=m->fn*m->dx[i]/m->totalstep;
                //print32_t .fx(i)
                
                double scale2=maxf[i]/m->fx[i];
                if (scale2<scale) scale=scale2;
            }
        }
        // update all speed
        
        m->fn=m->fn*scale;
        for (i=0;i< numaxis;i++){
            m->fx[i]=m->fx[i]*scale*m->sx[i];
            //xprintf("F%d:%f\n",i,m->fx[i]);
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
    m=&move[bpos];
    //color .col
    if (m->planstatus==1)return;
    //print32_t bpos

    double t,ac;
  double stepmm=stepmmx[m->fastaxis];
  ac=accel[m->fastaxis];
    m->ac1=ACCELL(m->fs,m->fn,ac);
    m->ac2=ACCELL(m->fn,m->fe,ac);
     
    
    
    m->rampup=ramplen(m->fs,m->fn,m->ac1,stepmm);
    m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);

    xprintf(PSTR("1\nRU:%d Rd:%d\n"),m->rampup,m->rampdown);
    xprintf(PSTR("FS:%f FN:%f FE:%f\n"),m->fs,m->fn,m->fe);
    
    //print32_t "Recalc"
    //unable to read nominal speed
    if (m->rampup>m->totalstep/2){
        m->fn=speedat(m->fs,m->ac1/stepmm,m->totalstep/2);
        m->ac1=ACCELL(m->fs,m->fn,ac);
        m->ac2=ACCELL(m->fn,m->fe,ac);
        m->rampup=m->totalstep/2;
        m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);
        //print32_t "x",.fn,.rampdown
        xprintf(PSTR("2\nRU:%d Rd:%d\n"),m->rampup,m->rampdown);
        xprintf(PSTR("FS:%f FN:%f FE:%f\n"),m->fs,m->fn,m->fe);
    }
    if (m->rampdown>m->totalstep/2){
        double f2;
        if (m->ac2>0){ 
            m->fe=speedat(m->fn,-m->ac2/stepmm,m->totalstep/2);
            xprintf(PSTR("3a\n"));
        } else {
            f2=speedat(m->fe,-m->ac2/stepmm,m->totalstep/2);
            if (f2<m->fn)m->fn=f2;
            xprintf(PSTR("3b %f\n"),f2);
             
        }
        m->ac1=ACCELL(m->fs,m->fn,ac);
        m->ac2=ACCELL(m->fn,m->fe,ac);
        m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);
        m->rampup=ramplen(m->fs,m->fn,m->ac1,stepmm);
        //print32_t "y",.fn
        xprintf(PSTR("3\nRU:%d Rd:%d\n"),m->rampup,m->rampdown);
        xprintf(PSTR("FS:%f FN:%f FE:%f\n"),m->fs,m->fn,m->fe);
    }
    if (m->rampup>m->totalstep) {
        // nothing work, so need to track back and reduce all speed along the way
        // for now, just maximize acceleration, force it
        m->ac1=accelat(m->fs,m->fe,m->totalstep)*stepmm;
        m->ac2=0;
        m->rampup=m->totalstep;
        m->rampdown=0;
        //print32_t "cek",
        xprintf(PSTR("4\nRU:%d Rd:%d\n"),m->rampup,m->rampdown);
        xprintf(PSTR("FS:%f FN:%f FE:%f\n"),m->fs,m->fn,m->fe);
    }
    m->planstatus=1;
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
    int32_t i,p;
    tmove *curr;
    tmove *prev;
  

    curr=&move[h];
    safespeed(curr);
    if (bufflen()<2) return;
    p=prevbuff(h);
    prev=&move[p];
  /* calculate jerk
                max_jerk
           x = -----------
                |v1 - v2|
        
           if x > 1: continue full speed
           if x < 1: v = v_max * x
    */
    double scale,scale2;
  double cjerk=jerk[curr->fastaxis];

    scale=1;
    for (i=0;i< numaxis;i++){
        double df=fabs(curr->fx[i]-prev->fx[i]);
        //xprintf("DFX%d:%f %f\n",i,prev->fx[i],curr->fx[i]);
        scale2=cjerk / df;
        if (scale2<scale)  scale=scale2;
    }
    //printf ("jerk scaling:%f\n",scale);

    if ((curr->status==1) && (prev->status!=0)) {
        //remove prev rampdown, curr rampup
        prev->fe=scale* (prev->fn+curr->fn)/2;//curr->fn; //
        //print32_t *(prev).fe,scale
        //*(prev).fe=scale*(*(curr).fn)
        prepareramp(p);
        //update current move again
        curr->fs=prev->fe;
    }
    
}

/*
  addmove
  =======
  Rutin menambahkan sebuah vektor ke dalam buffer gerakan
*/
double x1[numaxis],x2[numaxis];
void addmove(double f,double cx2,double cy2 ,double cz2 )
{
    needbuffer();
    tmove *m;
    m=&move[nextbuff(head)];
        m->col=9+(head & 7);
        x1[0]=cx1*stepmmx[0];
        x1[1]=cy1*stepmmx[1];
        x1[2]=cz1*stepmmx[2];
        x2[0]=cx2*stepmmx[0];
        x2[1]=cy2*stepmmx[1];
        x2[2]=cz2*stepmmx[2];
        m->fn=f;
        m->fe=0;
        m->fs=0;
        m->planstatus=0; //0: not optimized 1:fixed
        m->status=1; // 0: finish 1:ready 2:running
        //calculate delta
        int32_t ix;
        for (ix=0;ix< numaxis;ix++){
            int32_t delta=(x2[ix] - x1[ix]);
            m->dx[ix]=fabs(delta);
            
            if (delta>0) m->sx[ix]=1;
            else if (delta<0) m->sx[ix]=-1;
            else m->sx[ix]=0;
            m->cx[ix]=0;
            //xprintf("Dx%d:%d\n",ix,m->dx[ix]);
            //xprintf("Sx%d:%d\n",ix,m->sx[ix]);
            //xprintf("Cx%d:%d\n",ix,m->cx[ix]);
        }    
        for (ix=0;ix< numaxis;ix++){
            if ((m->dx[ix]>=m->dx[0]) && (m->dx[ix]>=m->dx[1]) && (m->dx[ix]>=m->dx[2]))  { 
                m->fastaxis=ix;
            }
        }
        m->totalstep=m->dx[m->fastaxis];
        xprintf(PSTR("Totalstep AX%d %d\n"),m->fastaxis,m->totalstep);
        
        // } buffer please
        
        // back planner
        head=nextbuff(head);
        planner(head);
    cx1=cx2;
    cy1=cy2;
    cz1=cz2;
}


tmove *m=0;
double tick,tickscale,fscale;
double x[numaxis]={0,0,0 };
double f,dl;
int32_t mctr;
uint32_t nextmicros;
int32_t motionrunning=0;

/*
 *  Inti dari motion loop 
 * 
 * 
 * 
 * 
 * 
 * 
 */

void motionloop(){
  feedthedog();
  if (!m ) {
        // start new move if available , if not exit
        if (!startmove()) return;
    }
    if (m->status==2) {
         // debug the micros
         if (0){
           xprintf(PSTR("N:%f M:%f\n"),nextmicros,micros());
         }
    if (nextmicros>micros()) return;
    if (m->totalstep){
        motionrunning=1;
        if (f>0)  
            dl=1/(f*stepmmx[m->fastaxis]);
        else 
            dl=1/stepmmx[m->fastaxis];

        tick=tick+dl;
        int32_t c=m->col;
    nextmicros=micros()+dl*timescale;
        //xprintf("Speed: %f\n",f);
        #if defined(__AVR__) || defined(ESP8266)
        // ESP8266 specific code here
        #else
 
        //pset (tick*tickscale+10,400-f*fscale),c
        putpixel (tick*tickscale+10,400-f*fscale,c);
        putpixel (x[0]/stepmmx[0]+50,x[1]/stepmmx[1]+40,c);
        //pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
        #endif
        int32_t ix;
        // bresenham work on motor step
        for (ix=0;ix< numaxis;ix++){
            m->cx[ix]=m->cx[ix]+m->dx[ix];
            if (m->cx[ix]>=m->totalstep) {
               x[ix]=x[ix]+m->sx[ix];
               mymotor[ix].stepping(m->sx[ix]);
               m->cx[ix]=m->cx[ix]-m->totalstep;
            } 
        }
        // next speed
        if (mctr<m->rampup) {
            //if (m->ac1>0 and f<.fn) or ( .ac1<0 and f>.fn) then 
            f=f+m->ac1*dl;
            if (f<1)f=1;
            //if ((m->ac1>0) && (f>m->fn))f=m->fn;
            //if ((m->ac1<0) && (f<m->fn))f=m->fn;
            //xprintf("Dl:%f\n",dl*timescale);
        }
        else if (mctr>m->totalstep-m->rampdown )
        {
            //if (.ac2>0 and f<=.fe) or ( .ac2<0 and f>=.fe) then 
            f=f+m->ac2*dl;
            if (f<1)f=1;
            //if ((m->ac2>0) && (f>m->fn))f=m->fn;
            //if ((m->ac2<0) && (f<m->fn))f=m->fn;
            //if (f>m->fn)f=m->fn;
            
        }
        else 
            f=m->fn;
        if (mctr % 63==0)xprintf(PSTR("F:%f\n"),f);
        mctr++;

    // delay (dl*F_CPU)
    // next timer
    // if finish call
    }
    if (mctr==m->totalstep){
          m->status=0;

            if (!startmove()){
                motionrunning=0;
                m=0;
            
            }
    }
        // 
        if (checkendstop) {
            for (int32_t e=0;e<numaxis;e++)
            {
                if (endstopstatus[e]) {
                    m->status=0;
                    motionrunning=0;
                    m=0;
                    break;
                }
            }
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
    for(int32_t e=0;e<numaxis;e++){
        if (x1[e]<0) endstopstatus[e]=1;else  endstopstatus[e]=0;
    }
#endif        
}
void homing(double x,double y,double z)
{
    m=0;
    head=tail;
    checkendstop=1;
    addmove(homingspeed,-100,-100,-100);
    startmove();
    waitbufferempty();
    // now slow down and check endstop once again
    cx1=cy1=cz1=0;
    double xx[numaxis]={0,0,0};
    for (int32_t e=0;e<numaxis;e++){
        // move away from endstop
        xx[e]=10;
        checkendstop=0;
        addmove(homingspeed,xx[0],xx[1],xx[2]);        
        waitbufferempty();
        // check endstop again slowly
        xx[e]=0;
        checkendstop=1;
        addmove(homingspeed/5,xx[0],xx[1],xx[2]);        
        waitbufferempty();
        // move away again
        xx[e]=2;
        checkendstop=0;
        addmove(homingspeed/5,xx[0],xx[1],xx[2]);        
        waitbufferempty();
    }
    checkendstop=0;
    cx1=x;
    cy1=y;
    cz1=z;
}
/*
  startmove
  Mulai menjalankan 1 unit di buffer gerakan terakhir
*/
int32_t startmove()
{
    if (head!=tail) {
        int32_t t=nextbuff(tail);
        m=&move[t];
    if (m->status==1) {
            prepareramp(t);
            tail=t;
            m->status=2;
            f=m->fs;
            if (f==0)  f=1;        
            mctr=0;
            nextmicros=micros()+100;
            motionrunning=1;
            xprintf(PSTR("RU:%d Rd:%d\n"),m->rampup,m->rampdown);
            xprintf(PSTR("FS:%f FN:%f FE:%f\n"),m->fs,m->fn,m->fe);
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
    motionloop(); 
    while (m){
        motionloop(); 
    }
}
/*
  needbuffer
  loop sampai da buffer yang bebas
*/
void needbuffer()
{
    if (bufflen()==numbuffer) {        
        xprintf(PSTR("Wait a buffer %d/%d\n"),bufflen(),numbuffer);
        while (bufflen()==numbuffer) {
            motionloop(); 
        }
    }
}
/*
  inisialisasi awal, wajib
*/
void initmotion(){


  cx1=0;
  cy1=0;
  cz1=0;
  lf=0;
  tick=0;
  tickscale=30;
  fscale=3;
  head=0;
  tail=0;           
  int32_t i;
  for (i=0;i<numaxis;i++){
    mymotor[i]=tmotor();
  }

  /*
   * pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);

   */}



