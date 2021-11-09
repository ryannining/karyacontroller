#pragma once
#ifndef fcurve_H
#define fcurve_H

int32_t sg, ok;
int32_t lsteps;


int32_t acup, acdn,a2,V;
int32_t s2,s4, s6;
int32_t  va, vi, vc, ve;

int32_t Sdest;
extern void readpixel2();
extern int pixelon;


int32_t isqrt(int32_t x) {
    if (x<1024)return 31;
    int64_t q = 1, r = 0;
    while ((q <<= 2) <= x) {}
    while (q > 1) {
        int64_t t;
        q >>= 2;
        t = x - r - q;
        r >>= 1;
        if (t >= 0) {
            x = t;
            r += q;
        }
    }
    //Serial.println(int32_t(r));
    return r;
}

#define iramplen(v0,v1,accel2) (v1-v0)/accel2
#define ramplenq(v0,v1,stepa) (v1-v0)*stepa
#define ramplenq2(v0,v1,stepj) ((v1-v0))*stepj

#define speedat(v0,a,s) (a * s  + v0)

void prepareramp(int32_t bpos)
{

  tmove *m, *next2, *next;
  m = &moves[bpos];
  //  f curve
  int32_t nve;
  vi = m->fs;
  if (bpos != (head)) {
    int npos = nextbuff(bpos);
    next = &moves[npos];
    if (npos != head) {
      next2 = &moves[nextbuff(npos)];
      nve = next2->fs;
    } nve = 0;
    //enext->fn=fmin(next->fn,next->fs + 0.5 * (nve - next->fs + next->delta));
    ve = next->fs;
  } else ve = 0;

  int32_t vcc = vi + (ve - vi + m->delta)/2;
  vc = fmin(m->fn, vcc); 

  //zprintf(PSTR("Pr %f %d %d\n"),ff(m->dis),fi(m->ac),fi(totalstep));

  float accel3f;
  int64_t accel3 = m->dis * m->ac;
  // velocity and acceleration are scaled to 4096
  // << 12
  // 
  acup = acdn = (accel3<<12) / totalstep; // 

  s2 = int64_t(totalstep) * (vc - vi) / accel3;
  s6 = int64_t(totalstep) * (vc - ve) / accel3;
  s4 = totalstep - (s2 + s6);


  //zprintf(PSTR("Pr %f %d %d\n"),ff(m->dis),fi(acup),fi(totalstep));

  sg = 0;
  ok = 0;
  Sdest = 0;
  V = vi<<12;

  m->status |= 4;

}


// curveloop is iterating each timestep (i set 0.001 sec) for changing velocity from constant jerk
// by change the initial a1x,a2 we can use this function for all 7 segment
// this mean the velocity only will change each 0.001 sec
// then in this timestep it check the lastS versus currentS in motor stepper steps
// it will move the motor (currentS - lastS) steps with current velocity
int curveloop() {
  if (sg != 2) {
		dlp=isqrt((V+=a2));    
  }
  //Serial.print(">");Serial.println(dlp);
  cmd0 = 1; //step command
  // bresenham movement on all laxis and set motor motion bit to cmd0
  bresenham(0); //
  bresenham(1);
  bresenham(2);
  if (NUMAXIS==4)bresenham(3);

  //Serial.println(dlp);
  // push T=CLOCK/V to timer command buffer
  cmd0 |= dlp << 5; // cmd0 is 32bit data contain all motor movement and the timing
  pushcmd();
  return --Sdest > 0; // continue until reach each segment distance , if reached, then need to initialize next segment
}

int coreloopscurve() {
 
  if (!ok) {
#ifdef output_enable
    if (sg == 0) {
      zprintf(PSTR("S1:%f  S2:%f S3:%f  S4:%f\n"), ff(s1),  ff(s2),  ff(s3),  ff(s4));
      zprintf(PSTR("S5:%f  S6:%f S7:%f\n"), ff(s5),  ff(s6),  ff(s7));
      zprintf(PSTR("V:%f A:%f J:%f\n"), ff(V), ff(a2), ff(a1x));
    }
#endif
    sg++;
    if (sg == 1){
      if (s2>0) {
        a2 = acup;
        Sdest = s2;
      } else sg++;
    }
    if (sg==2){
      if (s4>0) {
        V = vc<<12;
        dlp=isqrt(V);
        Sdest = s4;
      } else sg++;
    }
    if (sg==3){
      if (s6>0) {
      a2 = -acdn;
      Sdest = s6;
      } else  {
        //zprintf(PSTR("End segment\n"));
        m = 0;
        return 0;
      }
    }
  } 
  ok = curveloop();
  mctr--;
  return 1;
}
#endif
