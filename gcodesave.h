#if defined(ESP8266) || defined (ESP32)
extern int compress_loop();
extern void uncompress_loop();

extern void enduncompress();
extern void beginuncompress(String fn);
extern void dummy_beginuncompress(String fn);

extern void deletejob(String fn);
extern int ispause;
extern int uncompress;
extern int gcodepos,gcodesize;
extern float xMin,xMax,yMin,yMax,zMin,zMax;
#endif
