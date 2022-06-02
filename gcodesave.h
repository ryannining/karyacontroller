#pragma once


#define pack_pos(pos,f) (uint32_t(pos) || (uint32_t(f)>>24))
#define unpack_pos(pos) (uint32_t(pos) & 0b111111111111111111111111)
#define unpack_F(pos) uint8_t(pos<<24)
  
typedef struct {
  uint32_t pos; // 24 bit are position, 8 bit are F
  float px,py; // max start speed, maxcorner  
  float pz;
} tshapes;

extern int shapes_ctr;
#ifdef PREMIUMFEATURE
#define SHAPESNUM 300
#else
#define SHAPESNUM 10
#endif
extern tshapes shapes[SHAPESNUM+1];

#if defined(ESP8266) || defined (ESP32)
extern int compress_loop();
extern void uncompress_loop();

extern void enduncompress(bool force);
extern void beginuncompress(String fn,bool resume,int pos);
extern void prepareposition();
extern void dummy_beginuncompress(String fn);

extern void deletejob(String fn);
extern int ispause;
extern int uncompress;
extern int gcodepos, gcodesize;
extern float dMax, tMax, fMax, xMin, xMax, yMin, yMax, zMin, zMax;
#endif
