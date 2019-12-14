uint8_t cntF;
int prevacc;
int nextacc;
int curracc;
int scurve, has1, has2, has3, has4, has5, has6, has7;
int sg, ok;
int lsteps;


float s1, s2, s3, s4, s5, s6, s7;
float vi, vc, ve, vjerk1, vjerk7, mvjerk;
float acup, acdn, ja, a1x, a1, a2, as3, as7, T, V;
int Sdest;

#define jerk xyjerk

#define ramplenq(v0,v1,stepa) (v1-v0)*stepa
#define ramplenq2(v0,v1,stepj) ((v1-v0))*stepj

//#define speedat(v0,a,s,stp) (a * s / stp + v0)
#define speedat(v0,a,s) (a * s  + v0)

void prepareramp(int32_t bpos)
{

  tmove *m, *next;
  m = &move[bpos];
  scurve = (xyjerk > 0); // && (m->dis>5);
  // local m
  int faxis = FASTAXIS(m);
  // trapezoid ramps up /down
  float ru, rd;
  ru = rd = 0;
  ja = m->ac * 0.5;
  float accel2 = m->ac;
  float stepa = 1.0 / (accel2);
  float stepj;

  ru = ramplenq(m->fs, m->fn, stepa);
  rd = ramplenq(m->fe, m->fn, stepa);



  if (ru + rd > m->dis) {
    // if crossing and have rampup
    float r = ((ru + rd) - m->dis) * 0.5;
    ru -= r;
    rd -= r;
    if (ru < 0)ru = 0;
    if (rd < 0)rd = 0;
    if (rd > m->dis)rd = m->dis;
    if (ru > m->dis)ru = m->dis;
    m->fn = speedat(m->fs, accel2, ru);

    //    if (rd== 0)next->fs = m->fn;
  }

  // convert to steps
  float tosteps = totalstep / m->dis;
  ru = int(ru * tosteps);
  rd = int(rd * tosteps);
  if (scurve)stepj = tosteps / (m->ac);

  //if (m->fs - m->fe >= m->delta) m->fn = m->fs;
  prevacc = curracc;
  if (bpos != (head)) {
    next = &move[nextbuff(bpos)];
    if (int(next->fs - next->fe) >= int(next->delta)) next->fn = next->fs;

    if ((next->fs == next->fn) && (next->fe < next->fn)) { // all down
      nextacc = -1;
    } else if ((next->fe == next->fn) && (next->fs < next->fn)) { // all up
      nextacc = 1;
    } else if ((next->fe == next->fn) && (next->fs == next->fn)) { // all up
      nextacc = 0;
    } else {
      nextacc = 1;
    }
  } else {
    //fe=0;
    nextacc = 0;
  }

  has4 = int(m->delta) > int(fabs(m->fe - m->fs));

  if ((m->fs == m->fn) && (m->fe < m->fn)) { // all down
    curracc = -1;
  } else if ((m->fe == m->fn) && (m->fs < m->fn) && !has4) { // all up
    curracc = 1;
  } else if (m->fe == m->fn && m->fs == m->fn) { // all up
    curracc = 0;
  } else {
    if (m->fn > m->fe) curracc = -1; //else curracc = 0;
  }

  // lets check 1 by 1
  //
  // identify which segment is available
  //
  // lets check 1 by 1
  //
  mvjerk = scurve ? (0.5 * ja * ja / jerk) : 0;

  vjerk1 = 0;
  // if
  // segment 1, prevacc <=0 and fs<fn
  has1 = scurve && (prevacc <= 0) && (m->fs < m->fn);
  // segment 3, curracc <=0 and fs<fn
  has3 = scurve && (has4) && (m->fs < m->fn);

  if (has1)vjerk1 += mvjerk;
  if (has3)vjerk1 += mvjerk;
  // segment 2, constant acceleration prevacc <=0 and fs<fn
  //has2=(m->fn-m->fs>vjerk1*vjerk1);

  vjerk7 = 0;
  //
  // segment 1, prevacc <=0 and fs<fn
  has7 = scurve && (nextacc >= 0) && (m->fe < m->fn);
  // segment 3, curracc <=0 and fs<fn
  has5 = scurve && (curracc <= 0 && has4) && (m->fe < m->fn);
  if (!has4 && prevacc == -1 && curracc == -1)has5 = 0;



  if (has7)vjerk7 += mvjerk;
  if (has5)vjerk7 += mvjerk;
  // segment 2, constant acceleration prevacc <=0 and fs<fn
  has6 = (m->fn - m->fe > vjerk7 * vjerk7);



  vi = m->fs;
  vc = m->fn;
  ve = m->fe;
  float vi1 = sqrt(m->fs);
  float vc1 = sqrt(m->fn);
  float ve1 = sqrt(m->fe);

  //preparejerk(m->dis);
  float delta1, delta7;
  float jerk6 = 0.16667 * jerk;
  if (has1 || has3) {
    vjerk1 = fmin(vjerk1, fabs(vc1 - vi1));
    if (has1 && has3)vjerk1 = vjerk1 * 0.5;
    //zprintf(PSTR("Vj1:%f\n"),ff(vjerk1));
  }

  if (has1) {
    s1 = floor(ramplenq2(vi, sqr(vi1 + vjerk1), stepj));
  } else {
    s1 = 0;
  }
  if (has3) {
    s3 = floor(ramplenq2(sqr(vc1 - vjerk1), vc, stepj));
  } else {
    s3 = 0;
  }

  s2 = ru - s1 - s3;
  has2 = s2 > 0;
  if (!has2) {
    if (s2 < 0) {
      s2 = s1 + s3;
      s1 = floor(s1 * ru / s2);
      s3 = floor(s3 * ru / s2);
    }
    s2 = 0;
  }

  // calc new rate for up
  accel2 = accel2 / tosteps;


  if (ru) {
    acup = accel2 * (ru / (ru - 0.5 * (s1 + s3)));
  } else acup = accel2;

  // ===============================
  if (has5 || has7) {
    vjerk7 = fmin(vjerk7, fabs(vc1 - ve1));
    if (has5 && has7)vjerk7 = vjerk7 * 0.5;
  }

  if (has7) {
    s7 = floor(ramplenq2(ve, sqr(ve1 + vjerk7), stepj));
  } else {
    s7 = 0;
  }
  if (has5) {
    s5 = floor(ramplenq2(sqr(vc1 - vjerk7), vc, stepj));

  } else {
    s5 = 0;
  }
  s6 = rd - s5 - s7;
  has6 = s6 > 0;
  if (!has6) {
    if (s6 < 0) {
      s6 = s5 + s7;
      s5 = floor(s5 * rd / s6);
      s7 = floor(s7 * rd / s6);
    }
    s6 = 0;
  }// calc new rate for down
  if (rd) {
    acdn = accel2 * (rd / (rd - 0.5 * (s5 + s7)));
  } else acdn = accel2;

  s4 = totalstep - s1 - s2 - s3 - s5 - s6 - s7;
  has4 = s4 > 0;
  if (!has4) s4 = 0;


  sg = 0;
  ok = 0;
  Sdest = 0;
  V = vi;
  has1 = s1 > 0;
  has3 = s3 > 0;
  has5 = s5 > 0;
  has7 = s7 > 0;

  m->status |= 4;
  CORELOOP
}


// curveloop is iterating each timestep (i set 0.001 sec) for changing velocity from constant jerk
// by change the initial a1x,a2 we can use this function for all 7 segment
// this mean the velocity only will change each 0.001 sec
// then in this timestep it check the lastS versus currentS in motor stepper steps
// it will move the motor (currentS - lastS) steps with current velocity

int curveloop() {
  //dlp=(stepdiv2/sqrt(V)); // T = CPU_CLOCK/Vel
  if (sg!=4 && ((++cntF)&4))
	xInvSqrt(dlp, V);
  a2 += a1x; // jerk add to acceleration
  V += a2; // acceleration add to velocity
  // do bresenham this step longs
  //if (--mctr){
  cmd0 = 1; //step command
  // bresenham movement on all laxis and set motor motion bit to cmd0
  bresenham(0); //
  bresenham(1);
  bresenham(2);
  bresenham(3);

  // push T=CLOCK/V to timer command buffer
  cmd0 |= dlp << 5; // cmd0 is 32bit data contain all motor movement and the timing
  pushcmd();

  // lets save the data for display too
  //vv.push([sqrt(V),mi]);
  //}
  return --Sdest > 0; // continue until reach each segment distance , if reached, then need to initialize next segment
}

int coreloopscurve() {
  float lV = V;
  if (!ok) {
#ifdef output_enable
    if (sg == 0) {
      zprintf(PSTR("S1:%f  S2:%f S3:%f  S4:%f\n"), ff(s1),  ff(s2),  ff(s3),  ff(s4));
      zprintf(PSTR("S5:%f  S6:%f S7:%f\n"), ff(s5),  ff(s6),  ff(s7));
      zprintf(PSTR("V:%f A:%f J:%f\n"), ff(V), ff(a2), ff(a1x));
    }
#endif
    sg++;
    // initialize each segment
    if (sg == 1) { // constant jerk segment
      if (has1) {
        a2 = 0;
        a1x = acup / s1;
        Sdest = s1;
      } else sg++; // next segment
    }
    if (sg == 2) { // constant acc segment
      if (has2) {
        a2 = acup;
        //V=vi+(has1?vjerk1:0);
        a1x = 0;
        Sdest = s2;
      } else sg++;
    }
    if (sg == 3) { // constant jerk deceleration segment
      if (has3) {
        a2 = acup;
        //V=vc-vjerk1;
        a1x = -acup / s3;
        Sdest = s3;
      } else sg++;
    }
    if (sg == 4) { // constant velocity segment
      if (has4) {
        V=vc;
		xInvSqrt(dlp, V);
        a2 = 0;
        a1x = 0;
        Sdest = s4;
      } else sg++;
    }
    if (sg == 5) {
      if (has5) {
        a2 = 0;
        a1x = -acdn / s5;
        //V=vc-acdn*s5*0.5;
        Sdest = s5;
      } else sg++;
    }
    if (sg == 6) {
      if (has6) {
        a2 = -acdn;
        a1x = 0;
        //V=vc-(has5?acdn*s5*0.5:0);
        Sdest = s6;
      } else sg++;
    }
    if (sg == 7) {
      if (has7) {
        a1x = acdn / s7;
        a2 = -acdn;
        //V=ve+acdn*s7*0.5;//+a2*0.5; // needed to make sure we reach accurate exit velocity
        Sdest = s7;
      } else sg++;
    }
    if (sg == 8) {
      //zprintf(PSTR("End segment\n"));
      m = 0;
      return 0;
    }
  }
  ok = curveloop();
  mctr--;
  return 1;
}
