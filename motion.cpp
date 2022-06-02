
#include "common.h"
#include "gcode.h"

#include "timer.h"
#include "temp.h"
#include <stdint.h>


#include<Arduino.h>
#include "eprom.h"
#include "gcodesave.h"
#include "ir_remote.h"




#ifdef DEBUGLOOP
#define LOOP_IN(n) zprintf(PSTR("LOOP %d IN "),fi(n));
#define LOOP_OUT(n) zprintf(PSTR("LOOP %d OUT "),fi(n));
#else
#define LOOP_IN(n)
#define LOOP_OUT(n)
#endif // debugloop


#ifdef ESP8266
#define CLOCKCONSTANT 5000000.f        // tick/seconds
#define DSCALE 0   // use 5Mhz timer shift 0bit
#define DIRDELAY 2 // usec
#endif // esp

#ifdef  ESP32
#define CLOCKCONSTANT 80000000.f        // tick/seconds
#define DSCALE 0   // use 5Mhz timer shift 0bit
#define DIRDELAY 2 // usec
#endif // esp


#ifdef INVERTENDSTOP
#define ENDCHECK !
#else
#define ENDCHECK
#endif

#define ENDPIN INPUT_PULLUP


int MESHLEVELING = 0;
int atool_pin;
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
int32_t xycorner;
int i;
float ax_home[NUMAXIS];

uint32_t stepdiv2,stepdiv3,stepdiv2a,stepdiv3a;

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

#define SQRT sqrt


// LINEAR DRIVE IS SIMPLE
#define Cstepmmx(i) stepmmx[i]


// to calculate delay from v^2, fast invsqrt hack

#define sqrt32(n) sqrt(n)




int home_cnt = 0;
int limit_pin=-1;
void THEISR home_input() {
  home_cnt++;
  // tap home 3 times to start printing
}
void init_home_input() {
  home_cnt = 0;
  if (limit_pin>-1){
    pinMode(limit_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(limit_pin), home_input, CHANGE);
  }
}
void close_home_input() {
  if (limit_pin>-1){
    detachInterrupt(digitalPinToInterrupt(limit_pin));
  }
}


/*
  =================================================================================================================================================
  RESET_MOTION
  =================================================================================================================================================
  Reset all variable
*/
// keep last direction to enable backlash if needed
float perstepx, perstepy, perstepz;
extern void readconfigs();
void reset_motion()
{
  homingspeed = HOMINGSPEED;
  xycorner = XYCORNER;
  Lscale = LSCALE;
  ishoming = 0;
  cmctr = 0;
  e_multiplier = f_multiplier =  1;
  checkendstop = 0;
  endstopstatus = 0;

  head = tail = tailok = 0;
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  ocz1 = 0;
  ce01 = 0;
  extadv = 0;

#ifndef SAVE_RESETMOTION


/*
  maxf[MX] = XMAXFEEDRATE;
  maxf[MY] = YMAXFEEDRATE;
  maxf[MZ] = ZMAXFEEDRATE;
  maxf[nE] = E0MAXFEEDRATE;

  maxa[MX] = XACCELL;
  maxa[MY] = XACCELL * 0.5;//YMAXFEEDRATE / XMAXFEEDRATE;
  maxa[MZ] = XACCELL * 0.8;//ZMAXFEEDRATE / XMAXFEEDRATE;
  maxa[nE] = XACCELL;
*/


/*
  stepmmx[nE] = fabs(E0STEPPERMM);
  stepmmx[MX] = fabs(XSTEPPERMM);
  stepmmx[MY] = fabs(YSTEPPERMM);
  stepmmx[MZ] = fabs(ZSTEPPERMM);
  perstepx = 1.0 / (stepmmx[MX]);
  perstepy = 1.0 / (stepmmx[MY]);
  perstepz = 1.0 / (stepmmx[MZ]);

  odir[MX] = stepmmx[MX] < 0 ? -1 : 1;
  odir[MY] = stepmmx[MY] < 0 ? -1 : 1;
  odir[MZ] = stepmmx[MZ] < 0 ? -1 : 1;

  zaccel = maxa[MZ];//ZMAXFEEDRATE / XMAXFEEDRATE;
  accel = maxa[MX];

  //odir[nE] = E0STEPPERMM < 0 ? -1 : 1;



  ax_home[MX] = XMAX;
  ax_home[MY] = YMAX;
  ax_home[MZ] = ZMAX;

  xback[MX] = MOTOR_X_BACKLASH;
  xback[MY] = MOTOR_Y_BACKLASH;
  xback[MZ] = MOTOR_Z_BACKLASH;
  xback[nE] = MOTOR_E_BACKLASH;
*/

  axisofs[MX] = XOFFSET;
  axisofs[MY] = YOFFSET;
  axisofs[MZ] = ZOFFSET;
  axisofs[nE] = EOFFSET;

#endif
}
void pre_motion_set(){
  
  perstepx = 1.0 / (stepmmx[MX]);
  perstepy = 1.0 / (stepmmx[MY]);
  perstepz = 1.0 / (stepmmx[MZ]);

  odir[MX] = stepmmx[MX] < 0 ? -1 : 1;
  odir[MY] = stepmmx[MY] < 0 ? -1 : 1;
  odir[MZ] = stepmmx[MZ] < 0 ? -1 : 1;

  zaccel = maxa[MZ];//ZMAXFEEDRATE / XMAXFEEDRATE;
  accel = maxa[MX];
  
  xcheckevery = CHECKENDSTOP_EVERY * stepmmx[0];
  ycheckevery = CHECKENDSTOP_EVERY * stepmmx[1];
  zcheckevery = CHECKENDSTOP_EVERY * stepmmx[2];
  echeckevery = CHECKENDSTOP_EVERY * stepmmx[3];
}

int32_t mcx[NUMAXIS];
int32_t lctx, lcdx;
/*
  =================================================================================================================================================
  MOTOR CLASS
  =================================================================================================================================================
*/



#define FASTAXIS(n) ((n->status >> 4)&3)
//if(m)coreloop();
//#include "motors.h"

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


int32_t rampseg, rampup, rampdown;
#define rampv 1

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
  Bit 5 ... 31 Timing 27bit

  total 32 bit, i think for AVR 16 bit is enough ? to save ram

*/


/*
 planning to smoothing the command buffer
 stepdiv will be saved in dir command, with scaling to 250 precission
 in calculation clockconstant 5000000 * 1mm / 800step  = 6250 * 250 = 1562500  still less than 22 bit 
 
 and in motion we keep the velocity scale to 2500 (precission 0.0004)
 in calculation, 27 bit can handle velocity enough
 
 */
 


#define NUMCMDBUF 2047 // at least save motion for few milimeters



//#define nextbuffm(x) ((x) < NUMCMDBUF ? (x) + 1 : 0)
#define nextbuffm(x) ((x+1)&NUMCMDBUF)
#define nextbuffmV(x,v) ((x+v)&NUMCMDBUF)
#define prevbuffm(x) ((x-1)&NUMCMDBUF)



static volatile uint32_t cmddelay[(NUMCMDBUF + 1)], cmd0;
static volatile uint8_t   cmcmd, cmbit = 0;
int volatile cmhead = 0;
int volatile cmtail = 0;
int cmdlaserval = 0;

#define cmdfull2 (nextbuffm(cmhead)==prevbuffm(cmtail))
#define cmdfull (nextbuffm(cmhead)==cmtail)
#define cmdnotfull (nextbuffm(cmhead)!=cmtail)
#define cmdempty (cmhead==cmtail)
#define cmdlen (cmhead >= cmtail ? cmhead - cmtail : ((NUMCMDBUF+1) + cmhead) - cmtail)
static volatile uint32_t cmd_ctr=0;

#ifdef PLOTTING
#define NUMVEBUF 255
#define nextve(x) ((x+1)&NUMVEBUF)
#define prevve(x) ((x-1)&NUMVEBUF)
int ve_tail,ve_head;
int16_t ve_buff[NUMVEBUF+1];

void push_ve(int ve){ // ve scaled by 256
  int t=nextve(ve_head);
  ve_head=t;
  ve_buff[t]=ve;
}

String formatvelo(){
	String res;
	if (ve_head==ve_tail)return String("[]");
	res="[0";
	int t=ve_tail;
	while (t!=ve_head){
		t=nextve(t);
		res+=","+String(ve_buff[t]);
	}
	res+="]";
	return res;
	
}

void velo_loop(){
  extern uint32_t cmdve;  
  if (cmdve>0)push_ve(info_ve*10);
}
#endif

int nextok = 0, laserwason = 0;

int sendwait = 0; int delaywait = 1;

// we must interpolate delay here, between 10 steps
int ldelay = 0;
  
void waitloop();
static void pushcmd()
{
  // no more data if endstop hit
  if (checkendstop && (endstopstatus < 0))return;
  // wait until a buffer freed

  while (cmdfull) {
    waitloop();
    MEMORY_BARRIER();
  }

    

    cmhead = nextbuffm(cmhead);
    cmddelay[cmhead] = cmd0;
    cmd_ctr++;
}
uint32_t stepdiv4;
void newdircommand(int laserval)
{
  // 1  2  4  8  16 32 64 128  256   0-15    52488 
  // 0  1  2  3  4  5  6   7    8   9-12   13-31
  // 0  EX DX EY DY EZ DZ  EE  DE   --   STEPDIV2
  laserval=laserval>>2;
  if (laserval>63)laserval=63;
  cmd0 =(laserval)<<7;
  //zprintf(PSTR("int %d\n"), fi(laserval));
  if (sx[0] > 0)cmd0 |= 2;
  if (sx[1] > 0) {
    cmd0 |= 8;
#ifdef COPY_Y_TO_Z
    cmd0 |= 32;
#endif
  }
#ifndef COPY_Y_TO_Z
  if (sx[2] > 0)cmd0 |= 32;
#endif
  //if (sx[3] > 0)cmd0 |= 128;
  // TO know whether axis is moving
  if (bsdx[0] > 0)cmd0 |= 4;
#ifndef COPY_Y_TO_Z
  if (bsdx[2] > 0)cmd0 |= 64;
#endif
  if (bsdx[1] > 0) {
    cmd0 |= 16;
#ifdef COPY_Y_TO_Z
    cmd0 |= 64;
#endif
  }
  // axis E ?
  //if (bsdx[3] > 0)cmd0 |= 256;
  ldelay = 0;
  cmd0 |= uint32_t(stepdiv2 << 13);
  pushcmd();
  //cmd0 = (uint32_t(laserval & 255) | uint32_t(stepdiv2a<<8));
  //pushcmd();

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

#include "fcurve.h"


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
/*
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

}
*/

void backforward()
{
  int h;
  tmove *next, *curr;
  if ((h = head) == tailok) return;
  curr = 0;
  int lastok=tailok;  
  /*
  while (1) {
    next = curr;
    float fes = next ? next->fs : MINCORNERSPEED;
    curr = &moves[h];
    if (curr->fs != curr->maxs) {
     curr->fs = fmin(fes + curr->delta, curr->maxs); // maximum speed from next start
    }
   if ((h = prevbuff(h)) == tailok)break;
  }
  */
  float fes=MINCORNERSPEED;
  while (1) {
    next = curr;
    curr = &moves[h];
    if (curr->fs != curr->maxs) {
      curr->fs = fmin(fes + curr->delta, curr->maxs); // maximum speed from next start
    }
    fes = curr->fs;
   if ((h = prevbuff(h)) == tailok)break;
  }

  
 next = &moves[h=tailok];
  int ph=h;
  while ((h = nextbuff(h)) != head) {
    curr = next;
    next = &moves[h];
    if (curr->fs < next->fs) {
      float fs = curr->fs + curr->delta; // maximum speed at end of current move
      if (fs < next->fs) {
        next->fs = fs;
        tailok = ph;
      }
    }
    if (next->fs == next->maxs)tailok =  ph;
    ph=h;
  }

}


float mmdis[4];
tmove *pcurr,*pprev;
void planner(int32_t h)
{
  // mengubah semua cross feedrate biar optimal secepat mungkin
  // point to current moves
  pprev=pcurr;
  pcurr = &moves[h];
  // copy the last current vector (curru) to previous (prevu)
  // we want to update the current (curru)
  memcpy(prevu, curru, sizeof prevu);
  //curru[4] = 1; // ???
  // set the large first, we want to get the minimum acceleration and speed
  int32_t ac = accel;
  int32_t fn = pcurr->fn;
  int minaxis=0;
  for (int i = 0; i < 3; i++) {
    curru[i] = 0;
    if (pcurr->dx[i] != 0) {
      // vector at axis i is distance of the axis / distance total
      curru[i] = (mmdis[i] / pcurr->dis);
      // real accel and speed each axis, we want to know the maximum value of each axis, and get
      // the minimum from all axis , so no axis will move more than the maximum value configured
      ac = fmin(ac, fabs(maxa[i] / curru[i]));
      int32_t fx=fabs(maxf[i] / curru[i]);
      if (fx<fn){
        fn=fx;
        minaxis=i;
        //pcurr->maxv=maxf[i]/fx; // maximum speed multiplier
      }
    }
  }
  // xycorner is value configured at eeprom
  // calculate and save the delta speed from this acceleration
  pcurr->ac = ac << 1;
  pcurr->delta = fabs(pcurr->ac * pcurr->dis);
  // current speed is the minimum
  pcurr->fn = sqr(fn);
  pcurr->fr = fn;
  
  //**
  // ==================================
  // Centripetal corner max speed calculation, copy from other firmware
  // square_corner_velocity = sqrt(junction_deviation * max_accel * (sqrt(2) + 1))
  // 
  pcurr->maxs = MINCORNERSPEED;
  if (bufflen > 1) {
    float junc_cos = -prevu[MX] * curru[MX] - prevu[MY] * curru[MY] - prevu[MZ] * curru[MZ];
    if (junc_cos < 0.999) {
      pcurr->maxs=fmin(pcurr->fn, lastf);
      if (junc_cos > -0.999){
        float sin_theta_d2 = sqrt(0.5 * (1 - junc_cos)); // Trig half angle identity. Always positive.
// from klipper
        float R = (0.001 * xycorner * sin_theta_d2) / (1.0 - sin_theta_d2);
        // Approximated circle must contact moves no further away than mid-move
  /*
        if (pprev){
          float tan_theta_d2 = sin_theta_d2 / sqrt(0.5*(1.0+junc_cos))
          float move_cp_v2  = .5 * pcurr->dis * tan_theta_d2 * pcurr->ac;
          float pmove_cp_v2 = .5 * pprev->dis * tan_theta_d2 * pprev->ac;
          # Apply limits
          self.max_start_v2 = min(
              R * self.accel, R * prev_move.accel,
              move_centripetal_v2, prev_move_centripetal_v2,
              extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
              prev_move.max_start_v2 + prev_move.delta_v2)
          self.max_smoothed_v2 = min(
              self.max_start_v2
              , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)
        } else 
  */        
          pcurr->maxs = fmin( pcurr->maxs, R*pcurr->ac);
          // create an intermediate step between this 2 lines
      }
    }
  }
  lastf = pcurr->fn;

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
  //needbuffer();

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
  //if (babystep[3])ce01 -= babystep[3] * 0.001;
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

  // this x2 is local

  // calculate new distance each axis, in mm
  mmdis[0] = (cx2 - cx1) * odir[0];
  mmdis[1] = (cy2 - cy1) * odir[1];
  mmdis[2] = (cz2 - cz1) * odir[2];
  //mmdis[3] = (ce02 - ce01) * odir[3];

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



  // for LINEAR, just keep the INT value here
  x2[0] += (((int32_t)(cx2 * Cstepmmx(0)) - (int32_t)(cx1 * Cstepmmx(0))) ) ;
  x2[1] += (((int32_t)(cy2 * Cstepmmx(1)) - (int32_t)(cy1 * Cstepmmx(1))) ) ;
  x2[2] += (((int32_t)(cz2 * Cstepmmx(2)) - (int32_t)(cz1 * Cstepmmx(2))) ) ;


  x2[3] = 0;//(int32_t)(ce02 * e_multiplier * Cstepmmx(3)) - (int32_t)(ce01 * e_multiplier * Cstepmmx(3));

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
  // lets handle multiplier differently
  if (g0 || ishoming || ((x2[0] == 0) && (x2[1] == 0) && (x2[2] == 0)) ) {} else cf *= f_multiplier;


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
      //otx[3] = ce01;
      curr->status |= 128;
    }

    // Laser

    curr->laserval = (laserOn && (!g0) ? constantlaserVal : 0);
    head = nextbuff(head);
    curr->status |= 1; // 0: finish 1:ready
    // planner are based on cartesian coord movement on the motor
    planner(head);
    //#ifdef NONLINEAR
    // we keep the original target coordinate, to get correct stop position when a HARDSTOP performed
    cx1 = cx2; // save the target, not the original
    cy1 = cy2;
    cz1 = cz2;
    ocz1 = ocz2;
    ce01  = ce02;

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



static uint32_t cmdly;
uint32_t cmdve=0;
int cornerctr = 0;


/*
  =================================================================================================================================================
  MOTIONLOOP
  =================================================================================================================================================
*/
void otherloop(int r);
uint32_t dirbit, cm, ocm,  mctr2, dlmin, dlmax, cmd_mul;
int32_t timing = 0;
int32_t pwm_val;


// ======================================= COMMAND BUFFER ===========================================
int32_t laser_accum;
uint32_t laser_mul;
uint32_t cve,pve,ppve;
bool lastmove;
int lasermode=0;
// sigmoid const slowdown
const uint16_t PROGMEM slowdownDict[256]={
36,37,39,40,41,42,44,45,46,48,49,51,52,54,55,57,59,61,63,64,66,68,70,72,75,77,79,81,84,86,89,91,94,97,100,103,105,108,112,115,118,121,125,128,132,135,139,143,147,151,155,159,164,168,172,177,182,186,191,196,201,206,212,217,222,228,234,239,245,251,257,263,269,276,282,288,295,302,308,315,322,329,336,343,351,358,365,373,380,388,395,403,411,419,426,434,442,450,458,466,474,482,490,498,506,515,523,531,539,547,555,563,571,579,587,595,603,610,618,626,634,641,649,656,664,671,678,686,693,700,707,714,721,727,734,741,747,753,760,766,772,778,784,790,795,801,807,812,817,823,828,833,838,843,847,852,857,861,865,870,874,878,882,886,890,894,897,901,904,908,911,914,917,921,924,927,929,932,935,938,940,943,945,948,950,952,954,957,959,961,963,965,966,968,970,972,974,975,977,978,980,981,983,984,985,987,988,989,990,992,993,994,995,996,997,998,999,1000,1001,1002,1002,1003,1004,1005,1005,1006,1007,1008,1008,1009,1010,1010,1011,1011,1012,1012,1013,1013,1014,1014,1015,1015,1016,1016,1017,1017,1017,1018,1018,1018,1019,1019,1019,1020,1020,1020
};
//
//54,76,98,118,138,157,175,193,210,227,243,259,274,288,302,316,329,342,355,367,379,390,402,412,423,433,443,453,463,472,481,490,499,507,515,523,531,539,546,554,561,568,575,581,588,594,601,607,613,619,625,631,636,642,647,652,657,663,668,672,677,682,687,691,696,700,704,709,713,717,721,725,729,733,737,740,744,748,751,755,758,762,765,768,771,775,778,781,784,787,790,793,796,799,801,804,807,810,812,815,818,820,823,825,828,830,832,835,837,839,842,844,846,848,851,853,855,857,859,861,863,865,867,869,871,873,875,877,879,880,882,884,886,887,889,891,893,894,896,898,899,901,902,904,905,907,909,910,912,913,914,916,917,919,920,922,923,924,926,927,928,930,931,932,934,935,936,937,939,940,941,942,943,944,946,947,948,949,950,951,952,954,955,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,971,972,973,974,975,976,976,977,978,979,980,981,982,983,983,984,985,986,987,988,988,989,990,991,992,992,993,994,995,995,996,997,998,998,999,1000,1001,1001,1002,1003,1004,1004,1005,1006,1006,1007,1008,1008,1009,1010,1010,1011,1012,1012,1013,1014,1014,1015

bool FULLSPEED=true;
static THEISR void decodecmd()
{ 

  if (cmdempty) {
    //RUNNING = 1;
    if (lastmove) {
      if (lasermode)TOOL1(!TOOLON);
    }
    cmd_ctr = 0;
    lastmove = false;
    cmdve=cve=pve=ppve=0;
    return;
  }
  lastmove = true;

  // if on pause we need to decelerate until command buffer empty

  // to make sure the overrides speed is gradual, not sudden change
  //if (f_multiplier > f_rmultiplier)f_rmultiplier += 0.005;
  //else if (f_multiplier < f_rmultiplier)f_rmultiplier -= 0.005;


  uint32_t cmd = cmddelay[cmtail];
  // cmcmd is the 1st bit of the command
  cmcmd = cmd & 1;

  if (cmcmd) {
      // 1st bit is set then this is the motion command
      cmbit = (cmd >> 1) & 15;
      // if nothing to move then turn off laser , its end of the move
      // inform if non move is in the buffer
      //if (cmcmd && (cmbit==0))zprintf(PSTR("X"));
      
      ppve=pve;
      pve=cve;
      cve = (cmd >> 5);
      // 64 >> 1 + 64 >> 2 + 64 > 3
      //   32         16          8
      //   * 73 / 64 (shr 6)
      // stepdiv * 7 
      // fast interpolation beetwen moves
      cmdve = (((cmdve >>1 ) + (cve >>1 ) + (pve >> 2) + (ppve >> 3))*93)>>7;  
      // slow down based on the buffer left
      if (!FULLSPEED){
        if (cmd_ctr<256){
          //uint32_t vv;
          //vv=slowdownDict[cmd_ctr & 255];
          //vv=cmd_ctr<<2;
          if (cmd_ctr<=0) cmdve=128; else
          cmdve=128+(cmdve*cmd_ctr)>>8;
        } else // back to full speed
          if (RUNNING && !PAUSE) FULLSPEED=true; 
      }
      //Serial.print(stepdiv3);Serial.print("/");
      //Serial.println(cve/64.0);
      if (cmdve<16)cmdly=stepdiv3>>4; else cmdly = stepdiv3/cmdve;
      
  } else {
      // this is motion header command, contain the motor direction and the laser on/off
      cmbit = (cmd >> 1) & 255;
      dirbit=cmbit;
      cmdly = DIRDELAY;
      stepdiv3 = (cmd >> 13) << 6;  
      cmdlaserval = (cmd >> 7 ) & 63;
      stepdiv3a=stepdiv3>>8;

      laserwason = (Setpoint == 0) && (cmdlaserval>0);
      pwm_val = (Lscale*cmdlaserval);
      if (lasermode) {
        pwm_val = uint32_t(pwm_val*stepdiv3a)>>6;
        TOOL1(laserwason ? TOOLON : !TOOLON);
      }
  }
  

  nextok = 1;
  // point next command
  cmtail = nextbuffm(cmtail);
  cmd_ctr--;
  // laser = constant delay for laser
  if (lasermode)
    timer_set2((cmdlaserval<63)?pwm_val:cmdly,cmdly);
  else
    timer_set2(uint32_t(pwm_val*cmdly)>>6,cmdly);


#ifdef heater_pin
  HEATER(HEATING);
#endif

}

uint32_t mc, dmc, cmctr;
int32_t e_ctr = 0;
int e_dir, x_dir, y_dir, z_dir;
int mm_ctr = 0;
int32_t info_x_s, info_y_s, info_z_s;

void THEISR coreloopm()
{
  //servo_loop();
  //dmc=(micros()-mc); mc=micros();
  if (!nextok) {
    decodecmd();
    // nothing to decode, check again later, turn laser 1/4power
    #define MINDELAY 500
    timer_set2(uint32_t((lasermode?0:1)*MINDELAY*constantlaserVal*Lscale)>>8,MINDELAY);
    
    return;
  }

    // execute motion command
    if (cmcmd) { // 1: move
      mm_ctr++; // this counter used by probing to calculate real mm

      if (lasermode==2) {
#ifdef ANALOG_THC
       extern int thcspeed;
      if ((dirbit & 32)==0 && (mm_ctr & thcspeed)==0 && laserwason) {
        extern int thcdir,thcstep;
        if ((thcdir>-10) && (--thcstep>0)) {
          z_dir=thcdir;
          motor_2_DIR(z_dir);
          if (thcdir != 0)cmbit |= 4;
        }
      }
#endif
      }
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
        info_x_s -= x_dir;
        motor_0_STEP();
      }
      // AXIS 2
      if (cmbit & 2) {
        yctstep++;
        info_y_s -= y_dir;
        motor_1_STEP();
#if defined(COPY_Y_TO_Z) && !defined(motorz_servo)
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
        info_z_s -= z_dir;
      #ifndef motorz_servo
        motor_2_STEP();
      #endif
#endif
      }

      pinCommit();
      // after commit the PIN, send the LOW signal.
      // pincommit are used for shift register based pin, and its not used by real pin
      motor_3_UNSTEP();
      
      motor_0_UNSTEP();
      motor_1_UNSTEP();
      #ifndef motorz_servo
      motor_2_UNSTEP();
      #endif
      
      pinCommit();

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
      if (cmbit & 2) motor_0_DIR(x_dir = (cmbit & 1) ? -1 : 1);
#ifndef COPY_Y_TO_Z
      if (cmbit & 32) motor_2_DIR(z_dir = (cmbit & 16) ? -1 : 1);
#endif
      if (cmbit & 128) {
#ifndef DRIVE_XZY2
        motor_3_DIR(e_dir = (cmbit & 64) ? -1 : 1);
        //zprintf(PSTR("E:%d\n"), fi(e_dir));
#endif
      }
      if (cmbit & 8) {
        y_dir = (cmbit & 4) ? -1 : 1;
        motor_1_DIR(y_dir);
#if defined(COPY_Y_TO_Z) && !defined(motorz)
        motor_2_DIR(odir[2]*y_dir);
#endif
      }

      pinCommit();


    }
    // next command
    nextok = 0;
    decodecmd();
}


float ia = 0;
long firstdown;

int pixelon = 0;
void readpixel2() {

  //e_ctr=e_ctr&8191;
  char vv = g_str[e_ctr];
  vv &= ~32;
  if (vv == 'A')pixelon = 0; else pixelon = 1;
}

void doHardStop() {

#ifdef HARDSTOP
    float p = mctr;
    info_x = cx1 = info_x_s * perstepx;
    info_y = cy1 = info_y_s * perstepy;
    info_z = cz1 = ocz1 = info_z_s * perstepz;
    
    ce01 = 0;//m->dtx[3] - p * m->dx[3] / Cstepmmx(3);
    //zprintf(PSTR("Stopped!BUFF:%d\n"), fi(bufflen));
    extern void setXYZservo(float x, float y, float z);
    setXYZservo(info_x, info_y, info_z);
#endif

  endstopstatus = 0;
  checkendstop = 0;
  babystep[0] = babystep[1] = babystep[2] = babystep[3] = 0;
  lastf = MINCORNERSPEED;  
  m =0;
  pcurr =0;
  mctr = tailok = cmd_ctr = ok =   cmhead = cmtail = head = tail = 0;
  laserwason = 0;
}

int coreloop1()
{

  if (mctr <= 0) {
    return 0;
  }
  if (!coreloopscurve()) return 0;

  if (checkendstop || (!RUNNING)) {
    //      docheckendstop();

    if ((endstopstatus < 0) || (!RUNNING)) {

      //doHardStop();
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
  // prevent wait
  cm = micros();
#if defined(ESP32) || defined(ESP8266)
  feedthedog();
#endif  
  int  r=0;
  if (busy) return 0;

  if (!RUNNING || PAUSE){

  } else {
    busy = 1;

    if (!m ) {
      r=startmove();
    } else
    {
      r = coreloop1();
      nextmotoroff = cm;
      if (mctr <= 0) {
          // coreloop ended, check if there is still move ahead in the buffer
          //zprintf(PSTR("Finish:\n"));
          m->status = 0;
          m = pcurr = 0;
          r=startmove();
      }
    }
  }
  servo_loop();
  otherloop(0);
  busy = 0;
  return r;
}
long last_c0 = 0;

bool inwaitbuffer;

float info_x, info_y, info_z, info_e,info_ve;

void update_info(){
      info_x = info_x_s * perstepx;
      info_y = info_y_s * perstepy;
      info_z = info_z_s * perstepz;
      info_ve = cmdve*0.015625;
      #ifdef PLOTTING
      velo_loop();
      #endif
}

void otherloop(int r)
{

  if ((cm - last_c0 > 100000)) { // update every 20ms
    last_c0 = cm;
    update_info();
  }

  // this both check take 8us
  extern void thc_loop(uint32_t m);
  thc_loop(cm);
  temp_loop(cm);
#ifdef motortimeout
  if (!m   && (cm - nextmotoroff >= motortimeout)) {
    nextmotoroff = cm;
    power_off();
  }
#endif // motortimeout


  if (!wait_for_temp && !ishoming) {
    if (sendwait > 0) {
      sendwait -= delaywait;
      if (sendwait <= 0) {
        zprintf(PSTR("wait\n"));
        cmdve=cve=pve=ppve=0;
        delaywait = 1;
        //RUNNING = 1;
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


int empty1 = 0;
int32_t startmove()
{
  if (!RUNNING)return 0;
  if (NUMCMDBUF-cmd_ctr<3)return 0;

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
        if (lasermode)
          TOOL1(!TOOLON);
        sendwait = 1000000;

      }
    }
    return 0;
  } else empty1 = 0;
  sendwait = 0;
  // last calculation
  if (m ) return 0; 
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

  stepdiv2 = uint32_t(CLOCKCONSTANT * m->dis) / totalstep;
  stepdiv2a = uint32_t(CLOCKCONSTANT * m->dis) / (m->fr*totalstep); // maximum laser delay
  //Serial.println(stepdiv2);
  m->status &= ~3;
  m->status |= 2;


  mctr2 = mcx[0] = mcx[1] = mcx[2] = mcx[3] = 0; //mctr >> 1;
  tail = t;

  for (int i = 0; i < NUMAXIS; i++) {
    if (m->dx[i] > 0) {
      bsdx[i] = (m->dx[i]);
      sx[i] = 1;
    } else {
      bsdx[i] = -(m->dx[i]);
      sx[i] = -1;
    }
  }
  
  
  mctr = totalstep ;
  newdircommand(m->laserval);
  return 1;
}

/*
  =================================================================================================================================================
  DOCHECKENDSTOP
  =================================================================================================================================================
*/

void THEISR docheckendstop(int m)
{

  m = 1;
  if (limit_pin>-1) {
    if (m == 1) {
      int nc = 0;
      endstopstatus = 0;
      for (int d = 0; d < 3; d++) {
        if (ENDCHECK dread(limit_pin))nc++;
      }
      if (nc > 2)endstopstatus = -1;
    } else {
      endstopstatus = home_cnt > 0;
      home_cnt = 0;
    }
  }
}

/*
  =================================================================================================================================================
  HOMING
  =================================================================================================================================================
*/

String hstatus = "";
void homing()
{

  pause_pwm(true);
  IR_end();
  init_home_input();
  // clear buffer
  waitbufferempty();
  ocz1 = 0;
  cz1 = 0;
  ishoming = 1;
  addmove(100, fmin(cx1, 3), fmin(cy1, 3), ax_home[2] <= 1 ? 0 : ocz1, ce01);
  waitbufferempty();
  ishoming = 1;
  cx1 = 0;
  cy1 = 0;
  ocz1 = 0;
  ce01 = 0;
  endstopstatus = 0;
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
  hstatus = String(tx[0]);
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

  if (vx[0] > 0) cx1 = ax_home[0];
  else  cx1 = 0;
  if (vx[1] > 0) cy1 = ax_home[1];
  else  cy1 = 0;
  if (vx[2] > 0) ocz1 = ax_home[2];
  else  ocz1 = 0;
  cz1 = ocz1;

  //zprintf(PSTR("Home to:X:%f Y:%f Z:%f\n"),  ff(cx1), ff(cy1), ff(cz1));
  ishoming = 0;
  init_pos();


  pause_pwm(false);

}

/*
  =================================================================================================================================================
  PROBING
  =================================================================================================================================================
*/
float pointProbing(float floatdis = 0)
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
  addmove(homingspeed, 0, 0, zmm - floatdis, 0, 1, 1);
  zcheckevery = o;
  return zmm;
}

#ifdef MESHLEVEL
int XCount, YCount;
int ZValues[40][40]; //

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

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

void waitloop(){
    //extern uint32_t cm;
    //cm=micros();

    //if (busy)otherloop(0); else 
    motionloop();
    extern void important_loop();
    important_loop();
    
#ifdef output_enable1
    zprintf(PSTR("->H%d T%d cH%d cT%d\n"), fi(head), fi(tail), fi(cmhead), fi(cmtail));
#endif
}
void waitbufferempty(bool fullspeed)
{
  FULLSPEED=fullspeed;
  //if (head != tail)prepareramp(head);
  if (fullspeed){
    startmove();
  } else {
  }
  //#define output_enable1
#ifdef output_enable1
  zprintf(PSTR("Wait %d\n"), fi(mctr));
#endif
  LOOP_IN(2)
  inwaitbuffer = true;
  while ((head != tail) || m || (cmhead != cmtail) || (endstopstatus < 0)) { //(tail == otail) //
    if (fullspeed) {
      waitloop(); 
    } else {
      // command buffer is empty
      if (cmhead == cmtail)break; // if this pause loop, then break when no command moves
      feedthedog();
    }
    servo_loop();
    if (stopping)break;
    //if (PAUSE)break;
    //if (!RUNNING)break;
  }
  inwaitbuffer = false;
  cmdve=cve=pve=ppve=0;
  LOOP_OUT(2)
#ifdef output_enable1
  zprintf(PSTR("Empty"));
#endif
}


void faildetected()
{

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
  lastf = MINCORNERSPEED;
  head = 0;
  tail = 0;
  tailok = 0;
  cmhead = 0;
  cmtail = 0;
  cmd_ctr= 0;
#ifdef PLOTTING
  extern int ve_tail,ve_head;
  ve_tail=0;
  ve_head=0;
#endif
#ifdef ANALOG_THC
  extern int thctail,thchead;
  thctail=0;
  thchead=0;
#endif
  cmdve=cve=pve=ppve=0;
  info_e = ce01;
  info_x = cx1 = 0;
  info_y = cy1 = 0;
  info_z = cz1 = 0;
  info_x_s = 0;
  info_y_s = 0;
  info_z_s = 0;

}
void initmotion()
{

  reset_motion();




  motor_0_INIT();
  motor_1_INIT();
  motor_2_INIT();
  motor_3_INIT();
  motor_0_INIT2();
  motor_1_INIT2();
  motor_2_INIT2();
  motor_3_INIT2();


#ifdef POWERFAILURE
  pinMode(powerpin, INPUT_PULLUP);
  attachInterrupt(powerpin, faildetected, CHANGE);
#endif

  pinMotorInit
  atool_pin=tool1_pin;
  xpinMode(tool1_pin, OUTPUT);
  TOOL1(!TOOLON);

}
