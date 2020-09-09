#if defined(ESP8266) || defined (ESP32)
extern int compress_loop();
extern void uncompress_loop();

extern void enduncompress();
extern void beginuncompress(String fn);
extern int ispause;
extern int uncompress;
extern int gcodepos,gcodesize;
#endif
