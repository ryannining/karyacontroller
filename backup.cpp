/*
 * 


#ifdef DRIVE_DELTA
void addmoveDELTA(float cf, float cx2, float cy2 , float cz2, float ce02, int g0 , int rel)
{
  // create delta segments if needed


  int32_t dist;

  // deltas
  dist = approx_distance(labs(cx2 - cx1), labs(cy2 - cy1)); //,labs(diff.axis[Z]));
  if (dist < 2) {
    addmove(cf, cx2, cy2, cz2, ce02, g0, rel);
    return;
  }
  float df[4], cx[4];
  float sc = 1.0f / dist;
  df[0] = float(cx2 - cx1) * sc;
  df[1] = float(cy2 - cy1) * sc;
  df[2] = float(cz2 - cz1) * sc;
  df[3] = float(ce02 - ce01) * sc;



  int segment_total = dist;
  zprintf(PSTR("Dist:%d Segmen:%d\n"), fi(dist), fi(segment_total));

  if ((df[X] == 0 && df[Y] == 0))
  {
    addmove(cf, cx2, cy2, cz2, ce02, g0, rel);
    return;
  } else {

    //if you do all segments, rounding error reduces total - error will accumulate
    for (int s = 1; s < segment_total; s++) {

      addmove(cf, cx1 + (s * df[0]), cy1 + (s * df[1]), cz1 + (s * df[2]), ce01 + (s * df[3]), g0, rel);
    }
    //last bit to make sure we end up at the unsegmented target
    //segment = *target;
    //addmove(cf,cx2,cy2,cz2,ce02,g0,rel);
  }

}

#endif










 */
