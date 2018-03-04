/*
    =================================================================================================================================================
    DELTA PRINTER FUNCTIONS
    =================================================================================================================================================
*/


extern float sqrt7(float x);
extern float sqrt3(float x);





#ifdef USETIMER1
#define CORELOOP
#else
#define CORELOOP if(m)motionloop();
#endif


#ifdef NONLINEAR


// dummy step/mm for planner, 100 is good i think

#define FIXED2 100.f
#define IFIXED2 1.f/(FIXED2)
#define Cstepmmx(i) FIXED2

// 200 = on G0, 50 on G1 ~ 200 =2mm, 50 = 0.5mm
#define STEPPERSEGMENT 300 : 100

extern int32_t x2[4];

float sgx[NUMAXIS]; // increment delta each segment

float delta_diagonal_rod;
float DELTA_DIAGONAL_ROD_2;
float delta_radius = (DELTA_RADIUS );

float delta_tower1_x;
float delta_tower1_y;
float delta_tower2_x ;
float delta_tower2_y ;
float delta_tower3_x ;
float delta_tower3_y ;

extern float F_SCALE;



// maybe we should use fixed point to increase performance, or make all already multiplied with the steppermm (steppermm as the pixed point multiplicator)


/* ====================================================================================================== * 
 * 
 *  DELTA
 * ====================================================================================================== * 
 */

#if defined(DRIVE_DELTA)

#define NONLINEARHOME   cx1 = 0;  cy1  = 0;  cz1 = ax_max[2] * 0.1;
#define SINGLESEGMENT (ishoming || ((wm->dx[0] == 0) && (wm->dx[1] == 0)))
// only X and Y cause segmentation
#define STEPSEGMENT fmax(labs(wm->dx[0]),labs(wm->dx[1]))

void nonlinearprepare(){
  DELTA_DIAGONAL_ROD_2 = delta_diagonal_rod * delta_diagonal_rod;
  //delta_radius         = (DELTA_RADIUS );

  delta_tower1_x       = (COS(DegToRad(TOWER_X_ANGLE_DEG)) * delta_radius);
  delta_tower1_y       = (SIN(DegToRad(TOWER_X_ANGLE_DEG)) * delta_radius);
  delta_tower2_x       = (COS(DegToRad(TOWER_Y_ANGLE_DEG)) * delta_radius);
  delta_tower2_y       = (SIN(DegToRad(TOWER_Y_ANGLE_DEG)) * delta_radius);
  delta_tower3_x       = (COS(DegToRad(TOWER_Z_ANGLE_DEG)) * delta_radius);
  delta_tower3_y       = (SIN(DegToRad(TOWER_Z_ANGLE_DEG)) * delta_radius);

}
  
void transformdelta( float x, float y, float z) {
#ifdef output_enable
  zprintf(PSTR("transform delta "));
#endif
  if (ishoming)
  {
    // when homing, no transform
    x2[0] = int32_t(stepmmx[0] * x);
    x2[1] = int32_t(stepmmx[1] * y);
    x2[2] = int32_t(stepmmx[2] * z);

  }  else {

#ifdef ISPC
    float ex = x * graphscale;
    float ey = y * graphscale;
    float ez = z * graphscale;

    putpixel (ex + ey * 0.3 + 250, ey * 0.3 - ez + 250, 15);
#endif
    x2[0]     = stepmmx[0] * (sqrt(DELTA_DIAGONAL_ROD_2
                                   - sqr2(delta_tower1_x - x)
                                   - sqr2(delta_tower1_y - y)
                                  ) + z);
//    CORELOOP                              
    x2[1]     = stepmmx[1] * (sqrt(DELTA_DIAGONAL_ROD_2
                                   - sqr2(delta_tower2_x - x)
                                   - sqr2(delta_tower2_y - y)
                                  ) + z);
//    CORELOOP                              
    x2[2]     = stepmmx[2] * (sqrt(DELTA_DIAGONAL_ROD_2
                                   - sqr2(delta_tower3_x - x)
                                   - sqr2(delta_tower3_y - y)
                                  ) + z);
//    CORELOOP
    //F_SCALE=DELTA_DIAGONAL_ROD_2/(sqr2(delta_radius)+x*x+y*y);
  }
#ifdef output_enable
  zprintf(PSTR(": %f %f %f -> %d %d %d\n"), ff(x), ff(y), ff(z), fi(x2[0]), fi(x2[1]), fi(x2[2]));
#endif
}

/* ====================================================================================================== * 
 * 
 *  DELTASIAN
 * ====================================================================================================== * 
 */

#elif defined(DRIVE_DELTASIAN)

void nonlinearprepare(){
  DELTA_DIAGONAL_ROD_2 = delta_diagonal_rod * delta_diagonal_rod;

  delta_tower1_x       = -DELTA_RADIUS;
  delta_tower2_x       = +DELTA_RADIUS;
  delta_radius         = (DELTA_RADIUS );
}

#define NONLINEARHOME   cx1 = 0;  cy1  = 0;  cz1 = ax_max[2] * 0.1;
#define SINGLESEGMENT   (ishoming || (wm->dx[0] == 0))

// only X cause segmentation
#define STEPSEGMENT labs(wm->dx[0])

void transformdelta( float x, float y, float z) {
#ifdef output_enable
  zprintf(PSTR("transform delta "));
#endif
  if (ishoming)
  {
    // when homing, no transform
    x2[0] = int32_t(stepmmx[0] * x);
    x2[1] = int32_t(stepmmx[1] * y);
    x2[2] = int32_t(stepmmx[2] * z);

  }  else {
#ifdef ISPC
    float ex = x * graphscale;
    float ey = y * graphscale;
    float ez = z * graphscale;

    putpixel (ex + ey * 0.3 + 250, ey * 0.3 - ez + 250, 15);
#endif
    x2[0]     = stepmmx[0] * (sqrt(DELTA_DIAGONAL_ROD_2
                                   - sqr2(delta_tower1_x - x)
                                  ) + z);
//    CORELOOP                              
    x2[2]     = stepmmx[2] * (sqrt(DELTA_DIAGONAL_ROD_2
                                   - sqr2(delta_tower2_x - x)
                                  ) + z);
//    CORELOOP                              
    x2[1] = int32_t(stepmmx[1] * y);
  }
#ifdef output_enable
  zprintf(PSTR(": %f %f %f -> %d %d %d\n"), ff(x), ff(y), ff(z), fi(x2[0]), fi(x2[1]), fi(x2[2]));
#endif
}
#endif



#else // ELSE DRIVEDELTA
#define Cstepmmx(i) stepmmx[i]
#endif // DRIVEDELTA



