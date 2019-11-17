

// bikin pusing !, buat check TIMER di AVR, malah freeze

zprintf=console.log;
function PSTR(x){return x;}

function LOOP_IN(n) {zprintf(PSTR("LOOP %d IN "),fi(n));}
function LOOP_OUT(n) {zprintf(PSTR("LOOP %d OUT "),fi(n));}





// JERK Setting
var MINCORNERSPEED=1; // minimum cornering speed
var MINSTEP=0;
// Centripetal
//var JERK1 //centripetal corner , still not good, back to repetier style jerk
var DEVIATE=5;//;0.02*50 - for centripetal corner safe speed

// repetier 1
var JERK2=1;//repetier style jerk
//var JERK2A //repetier style jerk


var CLOCKCONSTANT=5000000.0;        // tick/seconds
var DSCALE=0;   // use 5Mhz timer shift 0bit
var DIRDELAY=20; // usec

var X=0;
var Y=1;
var Z=2;
var E=3;

var CNCMODE=1;
var MESHLEVELING = 0;
var vSUBPIXELMAX = 1;
var constantlaserVal = 0;
var laserOn = isG0 = 1;
var babystep[4] = [0, 0, 0, 0];
var homingspeed=0;
var xback=[0,0,0,0];
var head=tail=tailok=0;
var maxf=[0,0,0,0];
var xyscale=f_multiplier=e_multiplier=0;
var xyjerk=zjerk=accel=zaccel=0;
var i;
var mvaccel;
var ax_home[NUMAXIS];
var stepdiv, stepdiv2;
var wstepdiv2;
int32_t totalstep;
uint32_t bsdx[NUMAXIS];
int8_t  sx[NUMAXIS];
int32_t dlp, dl, dln, unms;
int8_t checkendstop, xctstep, yctstep, zctstep,  xcheckevery, ycheckevery, zcheckevery, echeckevery;
uint32_t ectstep = 0;
int16_t endstopstatus;
int8_t ishoming;
var axisofs[4] = {0, 0, 0, 0};
var F_SCALE = 1;
int8_t RUNNING = 1;
int8_t PAUSE = 0;
var stepmmx[4];
var odir[4] = {1, 1, 1, 1};
var retract_in, retract_out;
var retract_in_f, retract_out_f;
var cx1, cy1, cz1, ocz1, ce01, extadv;
tmove move[NUMBUFFER];

var sqr2(n) (n)*(n)


#include "nonlinear.h"

// to calculate delay from v^2, fast invsqrt hack
#ifdef __AVR__
//var FASTINVSQRT  // less precice speed faster 4us

var InvSqrt(var x)
{
#ifdef FASTINVSQRT
  int32_t* i = (int32_t*)&x;           // store floating-povar bits in integer
  *i = 0x5f335a86  - (*i >> 1);    // initial guess for Newton's method
#else // if FAST then bypass newtons method
  var xhalf = 0.5f * x;
  int32_t i = *(int32_t*)&x;            // store floating-povar bits in integer
  i = 0x5f375a86  - (i >> 1);    // initial guess for Newton's method
  x = *(float*)&i;              // convert new bits into float
  x = x * (1.5f - xhalf * x * x); // One round of Newton's method , disable if want faster but not precise
#endif
  return x;
}
#else // avr
// other CPU use normal SQRT
var InvSqrt(f) 1.0/sqrt(f)
#endif // avr

//var xInvSqrt(n) n>1?stepdiv2*InvSqrt(n):stepdiv

//var xInvSqrt(n) stepdiv2*InvSqrt(n)
//var xInvSqrt(n) n>0.25?stepdiv2*InvSqrt(n):2*stepdiv2

var pta = 0;
//var xInvSqrt(d,n) pta=(pta+n)*0.5;d=pta>0.25?stepdiv2*InvSqrt(pta):2*stepdiv2;
var xInvSqrt(d,n) d=n>0.25?stepdiv2*InvSqrt(n):2*stepdiv2;


var sqrt32(n) sqrt(n)


#ifdef ACT_KEY
var user_cnt = 0;
long lastms = 0;
void user_input_loop() {
  long m = millis();
  if (m - lastms > 2000) {
    lastms = m;
    if (user_cnt > 1) {
      zprintf(PSTR("USERKEY:%d\n"), fi(user_cnt));
      /*      switch (user_cnt) {
              case 4: homing(); break;
              //case 2: set_temp(0); break;
              case 5: set_temp(180); break;
        #ifdef ESP8266
              //case 3: if (uncompress)enduncompress(); else beginuncompress("/gcode"); break;
        #endif
              //case 1: set_temp(0); break;
            }
      */
      user_cnt = 0;
    }
  }
}
void user_input() {
  long m = millis();
  if (m - lastms > 500) {
    user_cnt++;
    lastms = m;
  }
  // tap home 3 times to start printing
}
void init_user_input() {
  attachInterrupt(digitalPinToInterrupt(limit_pin), user_input, FALLING);
}
void close_user_input() {
  detachInterrupt(digitalPinToInterrupt(limit_pin));

}
#else // 155
var user_input_loop()
var  init_user_input()
var close_user_input()
#endif // 155

#ifdef ISPC
void init_home_input() {};
void close_home_input() {};
#else
var home_cnt = 0;
void home_input() {
  home_cnt++;
  // tap home 3 times to start printing
}
void init_home_input() {
  home_cnt = 0;
  attachInterrupt(digitalPinToInterrupt(limit_pin), home_input, FALLING);
}
void close_home_input() {
  detachInterrupt(digitalPinToInterrupt(limit_pin));

}

#endif

/*
  =================================================================================================================================================
  RESET_MOTION
  =================================================================================================================================================
  Reset all variable
*/
// keep last direction to enable backlash if needed

void reset_motion()
{

  close_user_input();
  init_user_input();
  // 650bytes code
#ifdef SUBPIXELMAX
  vSUBPIXELMAX = SUBPIXELMAX;
#endif
  homingspeed = HOMINGSPEED;
  xyjerk = XYJERK;
  zjerk = fmin(homingspeed / 3, xyjerk);
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
  ocz1 = 0;
  ce01 = 0;
  extadv = 0;
  unms = 0;
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
  zaccel = accel * ZMAXFEEDRATE / XMAXFEEDRATE;

  mvaccel = XMOVEACCELL;

  stepmmx[0] = fabs(XSTEPPERMM);
  stepmmx[1] = fabs(YSTEPPERMM);
  stepmmx[2] = fabs(ZSTEPPERMM);
  stepmmx[3] = fabs(E0STEPPERMM);

  odir[0] = XSTEPPERMM < 0 ? -1 : 1;
  odir[1] = YSTEPPERMM < 0 ? -1 : 1;
  odir[2] = ZSTEPPERMM < 0 ? -1 : 1;
  odir[3] = E0STEPPERMM < 0 ? -1 : 1;


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



var FASTAXIS(n) ((n->status >> 4)&3)
//if(m)coreloop();
#include "motors.h"

var mb_ctr;
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


  24-4-2018, i change the math of ramp calculation using mm distance

* */
var currdis, prevdis, fe;

#ifdef NONLINEAR
var rampv;
var rampseg, rampup, rampdown;
#else
int32_t rampseg, rampup, rampdown;
var rampv 1
#endif
var sqr(x) x*x
var ramplenq(oo,v0,v1,stepa) if (v1>v0)oo=(v1-v0)*stepa;
//var speedat(v0,a,s,stp) (a * s / stp + v0)
var speedat(v0,a,s) (a * s  + v0)

/*var ramplen(oo,v0,v1,a,stepmm) oo=((int32_t)v1-(int32_t)v0)*stepmm/(a);
  var accelat(v0,v1,s) ((int32_t)v1  - (int32_t)v0 ) / (2 * s)



  var ramplenmm(oo,v0,v1,a) oo=(((int32_t)v1-(int32_t)v0)<<5)/(a);
  var speedatmm(v0,a,s) ((((int32_t)a * s)>>5)  + (int32_t)v0)
*/
void prepareramp(int32_t bpos)
{

  //var preprampdebug
  tmove *m, *next;
  m = &move[bpos];
  //if (m->status & 4)return; // already calculated

  var faxis = FASTAXIS(m);
  int32_t ytotalstep = labs(m->dx[faxis]);
var stepmm  Cstepmmx(faxis)

  var stepa = 1.0 / (m->ac);
  CORELOOP
  if (bpos != (head)) {
    next = &move[nextbuff(bpos)];
    fe = next->fs;
  } else fe = 0;
  var ru, rd;
  ru = rd = 0;
  ramplenq(ru, m->fs, m->fn, stepa);
  ramplenq(rd, fe, m->fn, stepa);

#ifdef preprampdebug
  zprintf(PSTR("\n========1========\nRU:%d Rd:%d Ts:%d\n"), rampup, rampdown, ytotalstep);
  zprintf(PSTR("FS:%f AC:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->ac), ff(m->fn),  ff(fe));
#endif


  if (ru + rd > m->dis) {
    // if crossing and have rampup
    var r = ((ru + rd) - m->dis) / 2;
    ru -= r;
    rd -= r;
    if (ru < 0)ru = 0;
    if (rd < 0)rd = 0;
    if (rd > m->dis)rd = m->dis;
    if (ru > m->dis)ru = m->dis;
    m->fn = speedat(m->fs, m->ac, ru);

    //    if (rd== 0)next->fs = m->fn;
  }

  CORELOOP

#ifdef preprampdebug
  zprintf(PSTR("========FINAL========\nRU:%d Rd:%d\n"), rampup, rampdown);
  zprintf(PSTR("FS:%f AC:%f FN:%f FE:%f\n"), ff(m->fs), ff(m->ac), ff(m->fn),  ff(fe));
#endif
  m->status |= 4;
  // convert to current
  rampup = ru * ytotalstep / m->dis;
  rampdown = rd * ytotalstep / m->dis;
  CORELOOP
}

/*
  =================================================================================================================================================
  PLANNER
  =================================================================================================================================================
  dipanggil oleh   untuk mulai menghitung dan merencakanan gerakan terbaik
  kontrol mbelok adalah cara mengontrol supaya saat menikung, kecepatan dikurangi sehingga tidak skip motornya

*/

var currf[5], prevf[5];
var lastf = 0;

void backforward()
{
  //zprintf(PSTR("bfplan. %d %d\n"), fi(tailok), fi(head));

  var h;
  tmove *next, *curr;
  // now back planner
  h = head; //
  if (h == tailok) return;

  // last data
  curr = &move[h];
  curr->fs = fmin(curr->maxs, (curr->ac * curr->dis));
  h = prevbuff(h);

  while (h != tailok) {
    next = curr;
    curr = &move[h];
    if (curr->fs != curr->maxs) {
      var fs = next->fs + (curr->ac * curr->dis);
      //var fs = next->fs + (accel * curr->dis);
      //zprintf(PSTR("back. %f\n"), ff(fs));
      if (fs < curr->maxs) {
        curr->fs = fs;
      } else {
        curr->fs = curr->maxs;
      }
    }
    h = prevbuff(h);

  }

  // forward
  //h=nextbuff(tailok);
  //if (h==head) {
  //    return;
  //}
  h = tailok;


  next = &move[h];
  h = nextbuff(h);
  while (h != head) {
    curr = next;
    next = &move[h];
    if (curr->fs < next->fs) {
      var fs = curr->fs + (curr->ac * curr->dis);
      //zprintf(PSTR("Forw. %f\n"), ff(fs));
      if (fs < next->fs) {
        next->fs = fs;
        tailok = h;
      }
      curr->fn = fmin(fs, curr->fn);
    }
    if (next->fs == next->maxs) tailok = h;
    h = nextbuff(h);
  }
  CORELOOP

}
var mmdis[4];
void planner(int32_t h)
{
  // mengubah semua cross feedrate biar optimal secepat mungkin
  int32_t p;
  tmove *curr;
  tmove *prev;

  curr = &move[h];
  var  scale = 1;
  int32_t xtotalstep = abs(curr->dx[FASTAXIS(curr)]);
  memcpy(prevf, currf, sizeof prevf);
  currf[4] = curr->fn;
  for (var i = 0; i < NUMAXIS; i++) {
    //prevf[i] = currf[i];
    currf[i] = 0;
    if (curr->dx[i] != 0) {

      var cdx = curr->fn * mmdis[i];
      if ((i != 3)) {
        var scale2 = float(maxf[i]) * curr->dis / fabs(cdx);
        if (scale2 < scale) scale = scale2;
      }
      // if Z then need to scale the acceleration too
      if (i == 2) {
        var scalea = zaccel * curr->dis / fabs(curr->ac * mmdis[i]);
        if (scalea < 1)curr->ac *= scalea;
#ifdef output_enable
        zprintf (PSTR("NEW AC :%f\n"), ff(curr->ac));
#endif
      }
      currf[i] = float(cdx) / curr->dis;

      CORELOOP
    }
  }
  var ujerk = xyjerk * scale;

  // update all speed and square it up, after this all m->f are squared !
  //zprintf (PSTR("Fratio :%f\n"), ff(scale));
#ifdef output_enable
  zprintf (PSTR("Fratio :%f\n"), ff(scale));
#endif
  scale *= curr->fn;
  curr->fn = (scale * scale);
#ifdef output_enable
  zprintf (PSTR("FN :%d\n"), fi(curr->fn));
#endif
  //var   1


  //if (bufflen() < 1) return;
  //p = prevbuff(h);
  //prev = &move[p];
  //if ((prev->status & 3) == 2)return;// if already planned

  // ==================================
  // Centripetal corner max speed calculation, copy from other firmware
  var max_f = MINCORNERSPEED * MINCORNERSPEED;
  if (bufflen() > 1) {

    /*

        p = prevbuff(h);
        prev = &move[p];

        var junction_unit_vec[3];
        var junction_cos_theta = 0.0;
        for (idx=0; idx<3; idx++) {
          junction_cos_theta -= prev.dx[idx]*curr.dx[idx]/(prev.dx[idx]*prev.dx[idx]);
          junction_unit_vec[idx] = curr.mm[idx]-prev.mm[idx];
        }

        // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
        if (junction_cos_theta > 0.999999) {
          //  For a 0 degree acute junction, just set minimum junction speed.
          block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
        } else {
          if (junction_cos_theta < -0.999999) {
            // Junction is a straight line or 180 degrees. Junction speed is infinite.
            block->max_junction_speed_sqr = SOME_LARGE_VALUE;
          } else {
            convert_delta_vector_to_unit_vector(junction_unit_vec);
            var junction_acceleration = limit_value_by_axis_maximum(settings.acceleration, junction_unit_vec);
            var sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // Trig half angle identity. Always positive.
            block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                           (junction_acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );
          }
        }
      }

      // Block system motion from updating this data to ensure next g-code motion is computed correctly.
      if (!(block->condition & PL_COND_FLAG_SYSTEM_MOTION)) {
        var nominal_speed = plan_compute_profile_nominal_speed(block);
        plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
        pl.previous_nominal_speed = nominal_speed;

        // Update previous path unit_vector and planner position.
        memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
        memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

        // New block is all set. Update buffer head and next buffer head indices.
        block_buffer_head = next_buffer_head;
        next_buffer_head = plan_next_block_index(block_buffer_head);


    */


    max_f = fmax(currf[4], prevf[4]);
#ifdef DRIVE_XYYZ
var MZ 1
var MY 2
#else
var MZ 2
var MY 1

#endif
    int32_t fdz = abs(currf[MZ] - prevf[MZ]);
#ifdef JERK2A
    var jerk = max_f * 0.7 * (1 - (currf[0] * prevf[0] + currf[1] * prevf[1] + currf[2] * prevf[2]) / ((currf[4] * prevf[4])));
#else
    int32_t fdx = currf[0] - prevf[0];
    int32_t fdy = currf[MY] - prevf[MY];

    var jerk = sqrt(fdx * fdx + fdy * fdy);

#endif
    CORELOOP
    var factor = 1;
    if (jerk > xyjerk) {
      factor = float(xyjerk) / jerk; // always < 1.0!
      CORELOOP
      //if (factor * max_f * 2.0 < xyjerk) {
      //    factor = xyjerk / (2.0 * max_f);
      CORELOOP
      //}
    }

    if (fdz > zjerk) {
      factor = fmin(factor, zjerk / fdz);
    }


    //if (jerk > XYJERK) max_f = XYJERK * max_f / jerk;
    //jerk = abs(fdz) ;
    //if (jerk > ZJERK) max_f = ZJERK * max_f / jerk;

    max_f = fmax(max_f * factor, MINCORNERSPEED);
    CORELOOP
#ifdef output_enable
    zprintf (PSTR("JERK:%d FACTOR:%f\n"), fi(jerk), ff(factor));
    zprintf (PSTR("XMFZ:%f\n"), ff(currf[2]));
    zprintf (PSTR("XMF:%d\n"), fi(max_f));
#endif

    max_f *= max_f;
  }
  //max_f=20;
  curr->maxs = fmin(curr->fn, lastf);
  curr->maxs = fmin(curr->maxs, max_f);
#ifdef output_enable
  zprintf(PSTR("maxs. %f\n"), ff(curr->maxs));
#endif
  lastf = curr->fn;
  //zprintf(PSTR(">> currfs,max %d %d\n"),fi(curr->fs),fi(curr->maxs));
  backforward();
  //zprintf(PSTR(">> currfs,max %d %d\n"),fi(curr->fs),fi(curr->maxs));


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
var fctr, tick;
var tickscale, fscale, graphscale;
#endif

int32_t  f;
var ta, acup, acdn, oacup, oacdn;
int32_t mctr, xtotalseg;
var fastaxis;
uint32_t nextmicros;
uint32_t nextdly;
uint32_t nextmotoroff;
var otx[NUMAXIS]; // keep the original coordinate before transform
int8_t repos = 0;

#ifdef USE_BACKLASH
var ldir[NUMAXIS] = {0, 0, 0, 0};
#endif

var lastmm[NUMAXIS]; // keep the original coordinate before transform
var currmm[NUMAXIS]; // keep the original coordinate before transform


void addmove(var cf, var cx2, var cy2, var cz2, var ce02, var g0 = 1, var rel = 0)
{


  needbuffer();



  int32_t x2[NUMAXIS];
  //XYSCALING
  //cx2*=xyscale;  cy2*=xyscale;
  if (head == tail) {
    //zprintf(PSTR("Empty !\n"));
  }
#ifdef output_enable
  zprintf(PSTR("\n\nADDMOVE\nTail:%d Head:%d \n"), fi(tail), fi(head));
  zprintf(PSTR("F:%f From X:%f Y:%f Z:%f E:%f\n"), ff(cf), ff(cx1), ff(cy1), ff(cz1), ff(ce01));
  zprintf(PSTR("To X:%f Y:%f Z:%f E:%f\n"),  ff(cx2), ff(cy2), ff(cz2), ff(ce02));
#endif
  tmove *curr;
  curr = &move[nextbuff(head)];
  curr->status = g0 ? 8 : 0; // reset status:0 planstatus:0 g0:g0

  cx1 -= babystep[0] * 0.001;
  cy1 -= babystep[1] * 0.001;
  cz1 -= babystep[2] * 0.001;
  ce01 -= babystep[3] * 0.001;
  if (rel) {
    cx2 += cx1;
    cy2 += cy1;
    cz2 += cz1;
    ce02 += ce01;
  }

  // mesh leveling
  var ocz2 = cz2;
  if (MESHLEVELING) {
    cz2 -= Interpolizer(cx2, cy2);
    zprintf(PSTR("ZI:%f\n"),  ff(cz2));
  }


#ifdef ISPC
  curr->col = 2 + (head % 4) * 4;
#endif
  // this x2 is local

  memcpy(lastmm, currmm, sizeof(currmm[0])*3);
  
  currmm[0]=mmdis[0] = (cx2 - cx1) * odir[0];
  currmm[1]=mmdis[1] = (cy2 - cy1) * odir[1];
  currmm[2]=mmdis[2] = (cz2 - cz1) * odir[2];
  mmdis[3] = (ce02 - ce01) * odir[3];

#ifdef output_enable

  zprintf(PSTR("Dis X:%f Y:%f Z:%f\n"),  ff(mmdis[0]), ff(mmdis[1]), ff(mmdis[2]));
#endif

  for (var i = 0; i < NUMAXIS; i++) {
    x2[i] = 0;
var deltax mmdis[i]


#ifdef USE_BACKLASH
    var dir;
    if (deltax < 0)dir = -1;
    else if (deltax > 0)dir = 1;
    else dir = 0;

    // if there is movement and have save last dir, and last dir <> current dir then add backlash
    if ((ldir[i] != 0) && (dir != 0) && (ldir[i] != dir)) {
      // add backlash steps to this axis
      //zprintf(PSTR("backlash %d %d\n"),fi(i),fi(xback[i]));
      //getch();
      // i think we should not doing backlash like this,
      // we should directly add to cmd buffer with step delay is
      var b = xback[i] * dir * 0.001;
      deltax += b;
      x2[i] = b * Cstepmmx(i);
    }
    // if no movement, then dont save direction
    if (dir != 0)ldir[i] = dir;
#endif
  }


#if defined( DRIVE_COREXY)
  curr->dis = sqrt(sqr2(mmdis[0] + mmdis[1]) + sqr2(mmdis[1] - mmdis[0]) + sqr2(mmdis[2]) + (rasterlen ? 0 : sqr2(mmdis[3])));
#elif defined( DRIVE_COREXZ)
  curr->dis = sqrt(sqr2(mmdis[0] + mmdis[2]) + sqr2(mmdis[2] - mmdis[0]) + sqr2(mmdis[1]) + (rasterlen ? 0 : sqr2(mmdis[3])));
#else
  curr->dis = sqrt(sqr2(mmdis[0]) + sqr2(mmdis[1]) + sqr2(mmdis[2]) + (rasterlen ? 0 : sqr2(mmdis[3])));
#endif

#ifndef NONLINEAR
  int32_t un = (1 << unms);
  x2[0] += (((int32_t)(cx2 * Cstepmmx(0)) - (int32_t)(cx1 * Cstepmmx(0))) * un) / un;
  x2[1] += (((int32_t)(cy2 * Cstepmmx(1)) - (int32_t)(cy1 * Cstepmmx(1))) * un) / un;
  x2[2] += (((int32_t)(cz2 * Cstepmmx(2)) - (int32_t)(cz1 * Cstepmmx(2))) * un) / un;


#else
  x2[0] += (cx2 - cx1) * Cstepmmx(0);
  x2[1] += (cy2 - cy1) * Cstepmmx(1);
  x2[2] += (cz2 - cz1) * Cstepmmx(2);
#endif




  CORELOOP
  // if rasterlen then step/mm is set to 32 to easily get the original position by shift register
  if (rasterlen) {

    if (g0)x2[3] = 0;
    else x2[3] = (int32_t)(ce02) - (int32_t)(ce01);
    //x2[3] += x2[3] > 0 ? -4 : 4;
  } else
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
  if (cf > 100)cf = 100; // prevent max speed
#endif

  CORELOOP
  curr->fn = cf; //curr->fn *= curr->fn;
  curr->fs = 0;
  //curr->planstatus = 0; //0: not optimized 1:fixed
  //calculate delta
  int32_t dd;
  dd = 0;
  var faxis = 0;
  // calculate the delta, and direction

#ifdef SHARE_EZ
  // if delta[Z] is not same direction with delta[E] then
  //  if (cz2 != cz1) ce02 = ce01; // zeroing the E movement if Z is moving and ZE share direction pin
var delt(i) x2[i]*odir[i]
  if ((delt(2) != 0) && (delt(2)*delt(3) < 0)) {
    ce02 = ce01; // zeroing the E movement if Z is moving and ZE share direction pin
    x2[3] = 0;
  }
#endif


  for (var i = 0; i < NUMAXIS; i++) {
    babystep[i] = 0;

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
    //curr->ac = 2 * (x2[2]==0?(g0 ? mvaccel : accel):50);
    curr->ac = 2 * (g0 ? mvaccel : accel);
    //zprintf(PSTR("F:%f A:%d\n"), ff(cf), fi(curr->ac));
    if (head == tail) {
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
    // we keep the original target coordinate, to get correct stop position when a HARD STOP performed
    cx1 = curr->dtx[0] = cx2; // save the target, not the original
    cy1 = curr->dtx[1] = cy2;
    cz1 = curr->dtx[2] = cz2;
    ocz1 = ocz2;
    ce01 = curr->dtx[3] = ce02;
    /*#else
      cx1 = cx2;
      cy1 = cy2;
      cz1 = cz2;
      ce01 = ce02;
      #endif
    */
  }

}





var N_ARC_CORRECTION 25
#ifdef ARC_SUPPORT

/*

  ARC using var implementation

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

void draw_arc(var cf, var cx2, var cy2, var cz2, var ce02, var fI, var fJ, var isclockwise)
{
var debug0
  uint32_t radius = approx_distance(abs(fI), abs(fJ));
  //   var acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc

  var cx = cx1 + fI ;
  var cy = cy1 + fJ ;
#ifdef debug1
  zprintf(PSTR("Arc  I, J,R:%f,%f,%d\n"), fI, fJ, radius);
  zprintf(PSTR("go cX cY:%f %f\n"), cx, cy);
#endif

  //uint32_t linear_travel = 0; //target[axis_linear] - position[axis_linear];
  var ne = ce01;
  var extruder_travel = (ce02 - ce01);
  var nz = ocz1;
  var z_travel = (cz2 - ocz1);

  var r_axis0 = -fI;  // Radius vector from center to current location
  var r_axis1 = -fJ;
  var rt_axis0 = cx2 - cx;
  var rt_axis1 = cy2 - cy;

#ifdef debug1
  sersendf_P(PSTR("go rX rY:%f %f\n"), r_axis0, r_axis1);
  sersendf_P(PSTR("go rtX rtY:%f %f\n"), rt_axis0, rt_axis1);
#endif

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  var angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
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

  var theta_per_segment = angular_travel / segments;
  var extruder_per_segment = (extruder_travel) / segments;
  var z_per_segment = (z_travel) / segments;

  var cos_T = 2.0 - theta_per_segment * theta_per_segment;
  var sin_T = theta_per_segment * 0.16666667 * (cos_T + 4.0);
  cos_T *= 0.5;

  var sin_Ti;
  var cos_Ti;
  var r_axisi;
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
    var nx = cx + r_axis0;
    var ny = cy + r_axis1;
    ne += extruder_per_segment;
    nz += z_per_segment;
    //move.axis[E] = extscaled>>4;
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
var jerkctr = 0;

#ifdef ISPC
var laseron = 0;
var LASER(x) {laseron=x==LASERON;}

var xs[4] = {0, 0, 0, 0};
var gx, gy, lx, ly;
var pcsx[4];
var graphstep(ix) xs[ix] +=pcsx[ix]
var lstepdiv = 1;
void dographics();
#endif

/*
  =================================================================================================================================================
  MOTIONLOOP
  =================================================================================================================================================
*/
void otherloop(var r);
uint32_t cm, ocm,  mctr2, dlmin, dlmax;
int32_t timing = 0;


/*
  =================================================================================================================================================
  COMMAND BUFFER
  =================================================================================================================================================
*/
#ifdef __AVR__
var NUMCMDBUF 100
#else
var NUMCMDBUF 100
#endif


var nextbuffm(x) ((x) < NUMCMDBUF-1 ? (x) + 1 : 0)

static volatile uint32_t cmddelay[NUMCMDBUF], cmd0;
static volatile var cmhead = 0, cmtail = 0, cmcmd, cmbit = 0;
static volatile uint32_t cmdlaserval = 0;

static int8_t mo = 0;
var cmdfull (nextbuffm(cmhead)==cmtail)
var cmdnotfull (nextbuffm(cmhead)!=cmtail)
var cmdempty (cmhead==cmtail)
static volatile var nextok = 0, laserwason = 0;

var sendwait = 0;

var maincmdlaserval = 0;
static THEISR void decodecmd()
{
  if (cmdempty) {

    return;

  }
  uint32_t cmd = cmddelay[cmtail];
  cmcmd = cmd & 1;
  if (cmcmd) {
    cmbit = (cmd >> 1) & 15;
    // if nothing to move then turn off laser , its end of the move
    // inform if non move is in the buffer
    //if (cmcmd && (cmbit==0))zprintf(PSTR("X"));
    cmdly = (cmd >> 5) >> DSCALE;
  } else {
    // header command,
    cmbit = (cmd >> 1) & 255;
    cmdly = DIRDELAY >> DSCALE;

    // if rasterlen then laser value get from raster data
    cmdlaserval = maincmdlaserval = (cmd >> 9) & 255;


    //else zprintf(PSTR("\nR"));
    //zprintf(PSTR("%d\n"), fi(cmdlaserval));
    //zprintf(PSTR("var %d\n"), fi(cmdlaserval));
  }
  //if (Setpovar == 0)LASER( !LASERON);
  //  if (maincmdlaserval && !rasterlen) {
  //zprintf(PSTR("%d\n"), fi(e_ctr));
  //    LASER(LASERON);
  //  };
  //zprintf(PSTR("%d"), fi(cmdlaserval));

  /*Serial.print("X ");
    Serial.print(cmd);
    Serial.print(":");
    Serial.print(cmdly);
    Serial.print(":");
    Serial.println(cmdlaserval);
  */
  nextok = 1;
  cmtail = nextbuffm(cmtail);
#ifdef USETIMER1
  timer_set(cmdly);
#ifdef heater_pin
  HEATER(HEATING);
#endif
#endif
  if (Setpovar == 0) {
    laserwason = maincmdlaserval > 0;
    if (laserwason && cmdlaserval) {
      // if laserval is 255 then we know its the full power / cutting
      if ((cmdlaserval < 255)) {

        var las = cmdlaserval / 4; //* (CLOCKCONSTANT / 8000000);
        //zprintf(PSTR("var %d\n"), fi(las));
        //delayMicroseconds(las);
        //LASER(!LASERON);
        //timer_set(cmdly-las*(CLOCKCONSTANT / 1000000));
#ifdef USETIMER1
        //timer_set2(cmdly, las);
#endif
      } else if (!rasterlen)LASER(LASERON);

    } else {
      // only change laser if CNCMODE is 0
      LASER( !LASERON);
    }
  }

}

uint32_t mc, dmc, cmctr;
var e_dir = 0;
int32_t e_ctr = 0;

var mm_ctr = 0;
//int16_t zmmm = 0;
void THEISR coreloopm()
{

  if (PAUSE) {
#ifdef USETIMER1
    timer_set(1000);
#endif
    return;
  }
  //dmc=(micros()-mc); mc=micros();
  if (!nextok) {
    decodecmd();
    return;
#ifndef USETIMER1
    if (!nextok) return;
#endif
  }
#ifdef USETIMER1
  {
#elif defined(ISPC)
  {
#else
  cm = micros();

  if (cm - nextmicros >= cmdly) {
    nextmicros = cm;
#endif

    if (cmcmd) { // 1: move
      mm_ctr++;

      if (rasterlen) { // only if RASTERMODE
        LASER( (cmbit & 8) ? LASERON : !LASERON);
      }
      //zmmm++;
      if (cmbit & 8) {
        ectstep += e_dir;
        motor_3_STEP();
      };

      if (cmbit & 1) {
        //cmctr++;
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
#ifdef __ARM__
      delayMicroseconds(2);
#endif
#ifdef ESP8266
      delayMicroseconds(2);
#endif
      motor_3_UNSTEP();
      motor_0_UNSTEP();
      motor_1_UNSTEP();
      motor_2_UNSTEP();
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
        if ((xctstep >= xcheckevery) || (yctstep >= ycheckevery) || (zctstep >= zcheckevery)) { // || (ectstep >= echeckevery) ) {
          xctstep = yctstep = zctstep = ectstep = 0;
          docheckendstop(0);
          if (endstopstatus < 0) {
            cmtail = cmhead;
            nextok = 0;
            return; // no more move and clear cmdbuffer if endstop hit/
          }
        }
      }

    } else { // 0: set dir

      e_dir = 0;
      if (cmbit & 2) motor_0_DIR((cmbit & 1) ? -1 : 1);
      if (cmbit & 8) motor_1_DIR((cmbit & 4) ? -1 : 1);
      if (cmbit & 32) motor_2_DIR((cmbit & 16) ? -1 : 1);
      if (cmbit & 128) {
        e_dir = (cmbit & 64) ? -1 : 1;
        motor_3_DIR(e_dir);
        //zprintf(PSTR("E:%d\n"), fi(e_dir));
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

//
/* idea:
  merge a non move
*/

var ldelay = 0;
static void pushcmd()
{
  // no more data if endstop hit
  if (checkendstop && (endstopstatus < 0))return;
  // wait until a buffer freed

  while (cmdfull) {
#ifdef USETIMER1
    MEMORY_BARRIER();
#endif
  }

  // if move cmd, and no motor move, save the delay
  if ( (cmd0 & 1) && !(cmd0 & (15 << 1))) {
    ldelay += cmd0 >> 5;
  } else {
    cmhead = nextbuffm(cmhead);
    if (cmd0 & 1)cmd0 += (ldelay << 5);
    cmddelay[cmhead] = cmd0;
    ldelay = 0;
  }

}

void newdircommand(var laserval)
{
  // change dir command
  //cmd0 = 0;//DIRDELAY << 6;
  cmd0 = (laserval << 9);
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
    //zprintf(PSTR("Odd delay:%d\n"), cmdly);
    return;
  }
  // this might be wrong because the last cmd maybe is from previous segment which have different step/mm
  var f = lstepdiv / cmdly;
  lstepdiv = stepdiv;
  f = sqrt(ta);
  //f =  sqrt(ta)/stepdiv2;
  tick = tick + cmdly; //1.0/timescale;
  int32_t c;
  // color by segment
  c = (tail & 3) + 10;
  if (jerkctr > 0)c = 1;
  // color by speed
  //c = (f - 20) * 4;

  //var cf = float(timescale) / dl;



  //pset (tick*tickscale+10,400-f*fscale),c
var realpos1(i) (xs[i] / stepmmx[i])
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
  setcolor(1);
  f = sqrt(ta);
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
  var ex = realpos1(0) * graphscale;
  var ey = realpos1(1) * graphscale;
  var ez = realpos1(2) * graphscale;

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
  var ex = realpos1(0) * graphscale;
  var ey = realpos1(1) * graphscale;
  var ez = realpos1(2) * graphscale;

  //putpixel (ex + 320, ey + 150, c);
  putpixel (ex + ey * 0.3 + 320, ey * 0.3 - ez + 150, c);
  //putpixel (ex + 320, ey  + 150, c);
#endif

#ifdef showlaser
  if (laseron) {
    putpixel (ex * 2 + 50 , ey * 2 + 50, 15);
  }// else     putpixel (ex * 5 , ey * 5, 9);
#endif

}
#else
var graphstep(ix)
#endif

/* ================================================================================================================
  BRESENHAM CORE

  this need to move to interrupt for smooth movement
  ================================================================================================================
*/



var bresenham(ix)\
  if ((mcx[ix] -= bsdx[ix]) < 0) {\
    cmd0 |=2<<ix;\
    mcx[ix] += totalstep;\
  }


#ifdef INTERPOLATEDELAY
var CALCDELAY dl = ((dl<<2)+(dlp-dl))>>2; // hack, linear interpolation of the delay
//var CALCDELAY dl = (dlp+dl)>>1; // hack, linear interpolation of the delay
#else
var CALCDELAY dl = dlp;
#endif
// ===============================
var ia = 0;
long firstdown;

var pixelon = 0;
static THEISR void readpixel2() {

  //e_ctr=e_ctr&8191;
  char vv = g_str[e_ctr];
  vv &= ~32;
  if (vv == 'A')pixelon = 0; else pixelon = 1;
}
var coreloop1()
{
#ifdef output_enable
  if (mctr == 10)zprintf(PSTR("DLY:%d \n"), fi(cmdly));
#else
  //if(mctr==10)zprintf(PSTR("DLY:%d \n"), fi(cmdly));
#endif

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
    if (rasterlen) {
      if ((mcx[3] -= bsdx[3]) < 0) {
        e_ctr += sx[3];
        readpixel2();
        mcx[3] += totalstep;
      }
      if (pixelon)cmd0 |= 2 << 3;
    } else bresenham(3);
    // if rasterlen and e is change then read the new laser value
    // next speed
var accele(a) ia=(ia+a)*0.5;ta+=ia
    if ((rampup -= rampv) > 0) {
      accele(acup);
      goto UPDATEDELAY;
    } else if ((rampdown -= rampv) < 0) {
      accele(acdn);
      // de-advance of the extruder
      if (firstdown) {
        mcx[3] += firstdown;
        firstdown = 0;
      }
UPDATEDELAY:
      if (ta < 1.05)ta = 1.05;
      //zprintf(PSTR("%d\n"),fi(ta));
      // if G0 update delay not every step to make the motor move faster
      //if (m->status & 8) {
#ifdef UPDATE_F_EVERY
      nextdly += dl;
      if (nextdly > UPDATE_F_EVERY) {
        nextdly -= UPDATE_F_EVERY;
        xInvSqrt(dlp, ta); //*F_SCALE;
      };
#else
      xInvSqrt(dlp, ta); //*F_SCALE;
#endif
    }
    CALCDELAY

    //if (dl<30)dl=30;
    cmd0 |= (dl) << 5;
    if (mctr < 20) {
      ///*
      //zprintf(PSTR("V:%f DL:%d\n"),ff(ta),fi(dl));
      //Serial.print(":");
      //Serial.println(dl);
      //*/
    }
    pushcmd();
    mctr--;
    /*    if (mctr==0){
      cmd0 = 1;
      pushcmd();
      }
    */
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
    if (checkendstop || (!RUNNING)) {
      //      docheckendstop();
      //zprintf(PSTR("%d \n"),fi(mctr));
      if ((endstopstatus < 0) || (!RUNNING)) {
        //zprintf(PSTR("Endstop hit"));
        if (!RUNNING) {
          // need to calculate at least correct next start position based on
          // mctr
#ifdef HARDSTOP
          var p = mctr;
          p /= totalstep;
          //p=1-p;
          cx1 = m->dtx[0] - p * m->dx[0] / Cstepmmx(0);
          cy1 = m->dtx[1] - p * m->dx[1] / Cstepmmx(1);
          cz1 = m->dtx[2] - p * m->dx[2] / Cstepmmx(2);
          ce01 = m->dtx[3] - p * m->dx[3] / Cstepmmx(3);
          zprintf(PSTR("Stopped!\n"));
#endif
        }
        endstopstatus = 0;
        checkendstop = 0;
        m->status = 0;
        mctr = 0;
        m = 0;
        head = tail;
        cmhead = cmtail;
        RUNNING = 1;
        return 0;
      }
    }
    return 1;
  }
  return 0;
}

var busy = 0;
// ===============================================
var cctr = 0;
var motionloop()
{
  if (cctr++ > 100000) {
    cctr = 0;
    //Serial.println(ndelay);
  }
  if (busy ) {
    zprintf(PSTR("Busy\n"));
    return 0;
  }
  busy = 1;
  CORELOOP
  var  r;
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

void otherloop(var r)
{
  user_input_loop();
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
        //zprintf(PSTR("Finish:\n"));
        m->status = 0;
        //if (m->fe == 0)f = 0;
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
    nextmotoroff = cm;
    power_off();
  }
#endif // motortimeout

#ifdef ESP8266
  feedthedog();
#endif


#endif // ispc

  if (!wait_for_temp && !ishoming) {
    if (sendwait > 0) {
      sendwait--;
      if (sendwait == 1) {
        zprintf(PSTR("wait\n"));
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
var subp = 1, laxis;

//var ts;
var xc[NUMAXIS];
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
var PSEGMENT(i) xc[i] -= xtotalseg * sgx[i];
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
  for (var i = 0; i < 4; i++) {
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
  for (var i = 0; i < 4; i++) {


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
  xInvSqrt(dln, (m->fn ));
#ifdef SUBPIXELMAX
  var u;
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
    acup /= u;
    acdn /= u;
    stepdiv2 /= u;
    stepdiv /= u;
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

#ifdef ISPC
  fctr = 0;
  m->col++;
#endif
}


int32_t startmove()
{

  if (cmdfull)return 0;
  if ((head == tail)) { // if empty buffer then wait a little bit and send "wait"
    //Serial.println("?");
    //m = 0; wm = 0; mctr = 0; // thi cause bug on homing delta
    if (!sendwait && (cmdempty)) {

      // send one time, is buffer is emspy after running
      if (Setpovar == 0) {
        LASER( !LASERON);
        laserwason = 0;
      }
#ifdef __ARM__
      sendwait = 800000;
#else
      sendwait = 600000;
#endif
    }
    return 0;
  }
  sendwait = 0;
  // last calculation
  if (m ) return 0; // if empty then exit
#ifdef output_enable
  zprintf(PSTR("\nSTARTMOVE\n"));
#endif
  // STEP 1
  var t = nextbuff(tail);
  // prepare ramp (for last move)
  prepareramp(t);
  m = &move[t];
  isG0 = m->status & 8;

  laxis = fastaxis;
  fastaxis = FASTAXIS(m);
  totalstep = labs(m->dx[fastaxis]);


  // recalculate acceleration for up and down, to minimize rounding error
  acup = acdn = 0;
  // 24-4-2018
  // Now need to convert mm rampup to step rampup

  //m->fs = ta;
  ta = m->fs;
  if (rampup)acup = float((m->fn  - m->fs )) / float(rampup); //(fmax(1,m->rampup-acramp));
  if (rampdown)acdn = -float((m->fn  - fe )) / float(rampdown); //(fmax(1,m->rampdown-acramp));

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

  // if from halt, then need to transform first point, if not then next first povar is previous point
  if (m->status & 128) {
    transformdelta(otx[0], otx[1], otx[2], otx[3]);
  }

  // STEP

  ts = 1.f / (xtotalseg);
  rampseg = (float)(totalstep) * ts;
  ts *= IFIXED2;

var CSGX(i) sgx[i] = float(m->dx[i]) * ts;
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
  m->status &= ~3;
  m->status |= 2;


  if (f == 0)
    nextmicros = micros();// if from stop

#ifdef output_enable
  //zprintf(PSTR("SUB:%d FS:%f TA:%f FE:%f RAMP:%d %d ACC:%f %f\n"), fi(subp), ff(m->fs), ff(m->fn), ff(fe), fi(rampup), fi(rampdown), ff(acup), ff(acdn));
#endif
  //  rampup = m->rampup  ;
  rampdown = totalstep - rampup - rampdown ;
  mctr2 = mcx[0] = mcx[1] = mcx[2] = mcx[3] = 0; //mctr >> 1;
  tail = t;

#ifdef output_enable

  zprintf(PSTR("Start tail:%d head:%d\n"), fi(tail), fi(head));
  zprintf(PSTR("RU:%d Rd:%d Ts:%d Dis:%f\n"), fi(rampup), fi(rampdown), fi(totalstep), ff(m->dis));
  zprintf(PSTR("FS:%f FN:%f AC:%f \n"), ff(m->fs), ff(m->fn), ff(m->ac));
  zprintf(PSTR("TA,ACUP,ACDN:%d,%d,%d \n"), fi(ta), fi(rampup), fi(rampdown));
  zprintf(PSTR("DX:%d DY:%d DZ:%d DE:%d \n"), fi(m->dx[0]), fi(m->dx[1]), fi(m->dx[2]), fi(m->dx[3]));

  //zprintf(PSTR("Last %f %f %f \n"), ff(px[0] / stepmmx[0]), ff(px[1] / stepmmx[0]), ff(px[2] / stepmmx[0]));
  //zprintf(PSTR("sx %d %d %d \n"), fi(sx[0]), fi(sx[1]), fi(sx[2]));
  //zprintf(PSTR("Status:%d \n"), fi(m->status));

#endif
  calculate_delta_steps();
#ifdef EXTADVANCE
  firstdown = fmin(extadv, ramdown) * totalstep;
  if (rampup) mcx[3] = -fmin(extadv, rampup) * totalstep;;
#endif
#ifdef SUBPIXELMAX
  rampup *= subp;
  rampdown *= subp;
#endif
  xInvSqrt(dlp, ta);
  dl = dlp;
#ifdef UPDATE_F_EVERY
  nextdly = UPDATE_F_EVERY + 100;
#endif


  // laser
  if (rasterlen) {
    if (isG0)e_ctr = m->dtx[3];
    if ((m->dx[3] != 0) && (!isG0)) {
      lastB--;

    }
    pixelon = 0;
  }
  return 1;
}

/*
  =================================================================================================================================================
  DOCHECKENDSTOP
  =================================================================================================================================================
*/
#ifdef INVERTENDSTOP
var ENDCHECK !
#else
var ENDCHECK
#endif

var ENDPIN INPUT_PULLUP

void docheckendstop(var m)
{

  // AVR specific code here
  // read end stop



#ifndef ISPC
  m = 1;
#ifdef limit_pin
  if (m == 1) {
    var nc = 0;
    endstopstatus = 0;
    for (var d = 0; d < 3; d++) {
      if (ENDCHECK dread(limit_pin))nc++;
    }
    if (nc > 2)endstopstatus = -1;
  } else {
    endstopstatus = home_cnt > 0;
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


void homing()
{
  close_user_input();
  init_home_input();
  // clear buffer
  addmove(100, 0, 0, ax_home[2] < 1 ? 0 : ocz1, ce01);
  waitbufferempty();
  ishoming = 1;
  cx1 = 0;
  cy1 = 0;
  ocz1 = 0;
  ce01 = 0;

  x2[0] = x2[1] = x2[2] = x2[3] = 0;

  int32_t stx[NUMAXIS];
  //  stx[0] =  stx[1] = stx[2] = stx[3] = 0;
  int32_t tx[NUMAXIS];
  //  tx[0] =  tx[1] = tx[2] = tx[3] = 0;
var mmax HOMING_MOVE
var smmax ENDSTOP_MOVE
  var vx[4] = { -1, -1, -1, -1};
  for (var t = 0; t < NUMAXIS; t++) {
    if (ax_home[t] > 1)vx[t] = 1;
    if (ax_home[t] < 1)vx[t] = 0;
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
  addmove(homingspeed, 0, tx[1], tx[2], 0, 1, 1);
  waitbufferempty();
  checkendstop = 0;
  addmove(homingspeed, 0, -stx[1], -stx[2], 0, 1, 1);
  waitbufferempty();
  checkendstop = 31;
  ishoming = 1;
  addmove(homingspeed, tx[0], 0, 0, tx[3], 1, 1);
#else
  checkendstop = 31;
  addmove(homingspeed, tx[0], tx[1], tx[2], tx[3], 1, 1);
#endif


  // now slow down and check endstop once again
  waitbufferempty();
  var xx[NUMAXIS];
var moveaway(e,F) {\
    if (vx[e]) {\
      xx[0]=xx[1]=xx[2]=xx[3]=0;\
      xx[e] =  - stx[e];\
      checkendstop = 0;\
      addmove(F, xx[0], xx[1], xx[2], xx[3],1,1);\
      waitbufferempty();\
    }\
  }
var xcheckendstop(e,F) {\
    xx[0] = xx[1] = xx[2] = 0; \
    xx[e] = tx[e]; \
    checkendstop = 1; \
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    waitbufferempty(); \
    xx[e] =  - stx[e] + axisofs[e]; \
    addmove(F, xx[0], xx[1], xx[2], xx[3], 1, 1); \
    waitbufferempty(); \
  }
  for (var e = 0; e < NUMAXIS; e++) {
    moveaway(e, homingspeed);
  }
  for (var e = 0; e < NUMAXIS; e++) {
    if (vx[e]) {
      // check endstop again fast
      xcheckendstop(e, 6);
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
  close_home_input();
  init_user_input();

}


/*
  =================================================================================================================================================
  PROBING
  =================================================================================================================================================
*/

var XCount, YCount;
var ZValues[10][10];

var pointProbing()
{
  close_user_input();

  // clear buffer
  waitbufferempty();
  mm_ctr = 0;
  int32_t tx;
var mmax HOMING_MOVE
var smmax ENDSTOP_MOVE
  tx = -20;

  // fast check every 31 step
  //zprintf(PSTR("Homing to %d %d %d\n"), tx[0], tx[1], tx[2]);

  // move away before endstop
  checkendstop = 1;
  var o = zcheckevery;
  zcheckevery = 1;
  addmove(homingspeed, 0, 0, tx, 0, 1, 1);


  // now slow down and check endstop once again
  waitbufferempty();
  var zmm = mm_ctr;
  //zprintf(PSTR("Probe %f,%f = %f\n"), ff(cx1), ff(cy1), ff(zmm));
  zmm /= stepmmx[2];
  checkendstop = 0;
  zcheckevery = o;
  //move back
  addmove(homingspeed, 0, 0, zmm, 0, 1, 1);
  init_user_input();
  return zmm;
}

void meshprobe(var sx, var sy, var tx, var ty, var mc) {

}
/*
  =================================================================================================================================================
  Z bilinear interpotale for mesh leveling
  =================================================================================================================================================
*/

// need to extrapolate if outside area
#ifdef MESHLEVEL
// crude interpolizer resolution is mm for x and y and 0.1mm for z
var Interpolizer(var zX, var zY) {

  //Indexes
  var X0i = 1;
  var Y0i = 1;


  //Interpolated values

  //Check the boundary, and extrapolate if necessary

  if (zX > ZValues[XCount][0])zX = ZValues[XCount][0];
  if (zY > ZValues[0][YCount])zY = ZValues[0][YCount];
  //if (zX < ZValues[1][0])zX = ZValues[1][0];
  //if (zY < ZValues[0][1])zY = ZValues[0][1];
  //Load the table data into the variables
  for (var i = 2; i < XCount; i++) {

    if (zX >= ZValues[i][0] && zX <= ZValues[i + 1][0]) {
      X0i = i;
    }
  }

  for (var i = 2; i < YCount; i++) {

    if (zY >= ZValues[0][i] && zY <= ZValues[0][i + 1]) {
      Y0i = i;
    }
  }


  var X0 = ZValues[X0i][0];
  var X1 = ZValues[X0i + 1][0];
  var Y0 = ZValues[0][Y0i];
  var Y1 = ZValues[0][Y0i + 1];

  var X0Y0 = ZValues[X0i][Y0i];
  var X0Y1 = ZValues[X0i][Y0i + 1];
  var X1Y0 = ZValues[X0i + 1][Y0i];
  var X1Y1 = ZValues[X0i + 1][Y0i + 1];

  //Performs the calculations - no optimization to save space

  var XMY0 = X0Y0 + (zX - X0) * (X1Y0 - X0Y0) / (X1 - X0);
  var XMY1 = X0Y1 + (zX - X0) * (X1Y1 - X0Y1) / (X1 - X0);

  return (XMY0 + (zY - Y0) * (XMY1 - XMY0) / (Y1 - Y0)) * 0.1;

}
#else
var Interpolizer(var  zX, var zY) {
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
#ifdef output_enable
  zprintf(PSTR("Wait %d\n"), fi(mctr));
#endif
  MEMORY_BARRIER()
  LOOP_IN(2)
  domotionloop
  while ((head != tail) || m || (cmhead != cmtail) || (endstopstatus < 0)) { //(tail == otail) //

    domotionloop
    servo_loop();
    MEMORY_BARRIER()
    //zprintf(PSTR("->%d\n"), fi(mctr));
  }
#ifdef laser_pin

  if (Setpovar == 0)LASER( !LASERON);
#endif
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
    zprintf(PSTR("Wait %d / %d \n"), fi(tail), fi(head));
#endif
    //wait current move finish
    var t = tail;
    LOOP_IN(3)
    MEMORY_BARRIER()
    while (t == tail) {
      domotionloop
      servo_loop();
      MEMORY_BARRIER()
      //zprintf(PSTR("%d\n"), fi(mctr));
    }
    //zprintf(PSTR("Done\n"));
    LOOP_OUT(1)

#ifdef output_enable
    zprintf(PSTR("Done %d / %d \n"), fi(tail), fi(head));
#endif
  }

}

void faildetected()
{
#ifdef POWERFAILURE
  // save the prvar last line only if printing using SDCARD
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
    //void addmove(var cf, var cx2, var cy2, var cz2, var ce02, var g0, var rel)
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
  dl  = 1000;


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

  //#ifdef LASERMODE
  xpinMode(laser_pin, OUTPUT);
  //#endif
  LASER( !LASERON);
  SPINDLE(!SPINDLEON);
#ifdef limit_pin
  pinMode(limit_pin, ENDPIN);
#endif
  pinMotorInit
#endif

}