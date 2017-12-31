#if defined(__AVR__) || defined(ESP8266)
// AVR specific code here
    //#include <avr/pgmspace.h>
    #include <arduino.h>
    void sendf_P(void (*writechar)(uint8_t), PGM_P format_P, ...);
    // No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.

  static void serial_writechar(uint8_t data) {
    Serial.write(data);
  }

    
    #define xprintf(...) sendf_P(serial_writechar, __VA_ARGS__)
#else

  #define PROGMEM
  #define PGM_P const char *
  #define PSTR(s) ((const PROGMEM char *)(s))
  #define pgm_read_byte(x) (*((uint8_t *)(x)))
  #define pgm_read_word(x) (*((uint16_t *)(x)))
  #define pgm_read_dword(x) (*((uint32_t *)(x)))
  #define xprintf printf
#endif

