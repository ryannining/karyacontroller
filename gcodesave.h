#ifdef ESP8266
extern int compress_loop();
extern void uncompress_loop();

extern void enduncompress();
extern void beginuncompress(String fn);
extern int ispause;
extern int uncompress;
#endif
