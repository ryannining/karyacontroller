#include "motion.h"
#include "timer.h"
#if defined(__AVR__)
// AVR specific code here
#include<arduino.h>
#elif defined(ESP8266)
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
    int enable;
    void stepping(int dx);
    
    
};

void tmotor::stepping(int dx)
{
  if (dx<0) {
  } else if (dx>0){
    
  }
}

tmotor mymotor[numaxis];
double jerk[numaxis]={20,20,5}; 
double accel[numaxis]={10,10,10};
double maxf[numaxis]={100,100,10};
double stepmmx[numaxis]={45,45,45};
tmove move[numbuffer+1];
double cx1,cy1,cz1,lf;


int bufflen(){
    
    int a=head-tail;
    if (a<0) a=a+numbuffer;
    return a;
}

/*
  safespeed 
  =========
  Bertujuan mencari kecepatan aman dari sebuah gerakan, misalnya gerakan dg speed 100, dan max speed 50, maka perlu diskala 0.5
  
*/
void safespeed(tmove *m) {
        int i;
    
        double  scale=1;
        
        for (i=0;i< numaxis;i++){
            if (m->dx[i]>0) {
                m->fx[i]=m->fn*m->dx[i]/m->totalstep;
                //print .fx(i)
                
                double scale2=maxf[i]/m->fx[i];
                if (scale2<scale) scale=scale2;
            }
        }
        // update all speed
        
        m->fn=m->fn*scale;
        for (i=0;i< numaxis;i++){
            m->fx[i]=m->fx[i]*scale*m->sx[i];
        }
        //print "w",.fx(1),.fx(2),.fx(3)
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

void prepareramp(int bpos)
{
    tmove *m;
    m=&move[bpos];
    //color .col
    if (m->planstatus==1)return;
    //print bpos

    double t,ac;
  double stepmm=stepmmx[m->fastaxis];
  ac=accel[m->fastaxis];
    m->ac1=ACCELL(m->fs,m->fn,ac);
    m->ac2=ACCELL(m->fn,m->fe,ac);
     
    
    
    m->rampup=ramplen(m->fs,m->fn,m->ac1,stepmm);
    m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);
    
    //print .rampup,.rampdown,.totalstep,.fs,.fn,.fe
    //print "Recalc"
    //unable to read nominal speed
    if (m->rampup>m->totalstep/2){
        m->fn=speedat(m->fs,m->ac1/stepmm,m->totalstep/2);
      m->ac1=ACCELL(m->fs,m->fn,ac);
        m->ac2=ACCELL(m->fn,m->fe,ac);
        m->rampup=m->totalstep/2;
        m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);
        //print "x",.fn,.rampdown
    }
    if (m->rampdown>m->totalstep/2){
        double f2;
        if (m->ac2>0){ 
            m->fe=speedat(m->fn,m->ac2/stepmm,m->totalstep/2);
        } else {
            f2=speedat(m->fe,-m->ac2/stepmm,m->totalstep/2);
            if (f2<m->fn)m->fn=f2;
             
        }
        m->ac1=ACCELL(m->fs,m->fn,ac);
        m->ac2=ACCELL(m->fn,m->fe,ac);
        m->rampdown=ramplen(m->fe,m->fn,m->ac2,stepmm);
        m->rampup=ramplen(m->fs,m->fn,m->ac1,stepmm);
        //print "y",.fn
    }
    if (m->rampup>m->totalstep) {
        // nothing work, so need to track back and reduce all speed along the way
        // for now, just maximize acceleration, force it
        m->ac1=accelat(m->fs,m->fe,m->totalstep)*stepmm;
        m->ac2=0;
        m->rampup=m->totalstep;
        m->rampdown=0;
        //print "cek",
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
void planner(int h)
{
    // mengubah semua cross feedrate biar optimal secepat mungkin
    if (bufflen()<2) return;
    int i,p;
    tmove *curr;
    tmove *prev;
  

    p=prevbuff(h);
    prev=&move[p];
    curr=&move[h];
    safespeed(curr);
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
        scale2=cjerk / fabs(curr->fx[i]-prev->fx[i]);
        if (scale2<scale)  scale=scale2;
    }

    if ((curr->status==1) && (prev->status!=0)) {
        //remove prev rampdown, curr rampup
        prev->fe=scale* curr->fn; //(*(prev).fn+*(curr).fn)/2
        //print *(prev).fe,scale
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
        m->col=1+(head & 7);
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
        int ix;
        for (ix=0;ix< numaxis;ix++){
            double delta=(x2[ix] - x1[ix]);
            m->dx[ix]=fabs(delta);
            m->sx[ix]=1;
            if (delta<0) m->sx[ix]=-1;
            m->cx[ix]=0;
        }    
        for (ix=0;ix< numaxis;ix++){
            if ((m->dx[ix]>=m->dx[0]) && (m->dx[ix]>=m->dx[1]) && (m->dx[ix]>=m->dx[2]))  { 
                m->fastaxis=ix;
            }
        }
        m->totalstep=m->dx[m->fastaxis];
        
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
int i;
uint32_t nextmicros;
int motionrunning=0;

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
  if (!m) {
        // start new move if available , if not exit
        if (!startmove()) return;
    }
    if (m->status==2) {
         // debug the micros
         if (0){
           #if defined(__AVR__)
          // AVR specific code here
           Serial.printf("N:%f M:%f\n",nextmicros,micros());
          #elif defined(ESP8266)
          // ESP8266 specific code here
           Serial.print("N:");
           Serial.print(nextmicros);
           Serial.print("M:");
           Serial.print(micros());
           Serial.print("\n");
          #else
           printf("N:%f M:%f\n",nextmicros,micros());
           #endif
         }
    if (nextmicros>micros()) return;
    motionrunning=1;
        if (f>0)  
            dl=1/(f*stepmmx[m->fastaxis]);
        else 
            dl=1/stepmmx[m->fastaxis];

        tick=tick+dl;
        int c=m->col;
    nextmicros=micros()+dl*timescale;
        #if defined(__AVR__)
        // AVR specific code here
        #elif defined(ESP8266)
        // ESP8266 specific code here
        Serial.write("Speed:");
        Serial.print(f);
        Serial.write("\n");
        #else
 
        //pset (tick*tickscale+10,400-f*fscale),c
        putpixel (tick*tickscale+10,400-f*fscale,c);
      putpixel (x[0]/stepmmx[0]+50,x[1]/stepmmx[1]+40,c);
        //pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
        #endif
        int ix;
        // bresenham work on motor step
        for (ix=0;ix< numaxis;ix++){
            m->cx[ix]=m->cx[ix]+m->dx[ix];
            if (m->cx[ix]==m->totalstep) {
               x[ix]=x[ix]+m->sx[ix];
               mymotor[ix].stepping(m->sx[ix]);
               x1[ix]+=m->sx[ix];
               m->cx[ix]=m->cx[ix]-m->totalstep;
            } 
        }
        // next speed
        if (i<=m->rampup)
            //if (m->ac1>0 and f<.fn) or ( .ac1<0 and f>.fn) then 
            f=f+m->ac1*dl;
        else if (i>=m->totalstep-m->rampdown )
            //if (.ac2>0 and f<=.fe) or ( .ac2<0 and f>=.fe) then 
            f=f+m->ac2*dl;
        else 
            f=m->fn;
    i++;

    // delay (dl*F_CPU)
    // next timer
    // if finish call
    if (i>m->totalstep){
          m->status=0;

            if (!startmove()){
                motionrunning=0;
                m=0;
            
            }
    }
        // 
        if (checkendstop) {
            for (int e=0;e<numaxis;e++)
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

int docheckendstop()
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
    for(int e=0;e<numaxis;e++){
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
    for (int e=0;e<numaxis;e++){
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
int startmove()
{
    if (head!=tail) {
        int t=nextbuff(tail);
        m=&move[t];
    if (m->status==1) {
            prepareramp(t);
          tail=t;
            m->status=2;
            f=m->fs;
            if (f==0)  f=1;        
            i=0;
            nextmicros=micros()+100;
            motionrunning=1;
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
    printf("Wait a buffer\n");
    motionloop(); 
    while (bufflen()==numbuffer) {
        motionloop(); 
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
  tickscale=5;
  fscale=1;
  head=0;
  tail=0;           
  int i;
  for (i=0;i<numaxis;i++){
    mymotor[i]=tmotor();
  }
}



