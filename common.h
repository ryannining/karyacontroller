#if defined(__AVR__) || defined(ESP8266)
// AVR specific code here
    //#include <avr/pgmspace.h>
    #include <arduino.h>
static void serial_writechar(uint8_t data) {
Serial.write(data);
}


void sendf_P(PGM_P format_P, ...);
// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.



//#define xprintf(...) sendf_P(serial_writechar, __VA_ARGS__)
//#define sersendf_P(...) sendf_P(serial_writechar, __VA_ARGS__)
#define xprintf sendf_P
#define sersendf_P sendf_P

#else
  #include<stdio.h>
  #include<stdint.h>
  #define PROGMEM
  #define PGM_P const char *
  #define PSTR(s) ((const PROGMEM char *)(s))
  #define pgm_read_byte(x) (*((uint8_t *)(x)))
  #define pgm_read_word(x) (*((uint16_t *)(x)))
  #define pgm_read_dword(x) (*((uint32_t *)(x)))

static void serial_writechar(uint8_t data) {
    printf("%c",(char)data);
#define xprintf printf
#define sersendf_P printf

}
#endif

