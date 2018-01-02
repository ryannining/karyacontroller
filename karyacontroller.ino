
#include "common.h"
#include "gcode.h"
void setup() {
  // put your setup code here, to run once:
  initmotion();
  init_gcode();
  //Serial.setDebugOutput(true);
  Serial.begin(115200);
}
extern void demo();
extern void motionloop();
int line_done, ack_waiting = 0;
void gcode_loop() {
  //double x=12.345;
  //xprintf(PSTR("Motion demo %d %f\n"),10,x);
  //delay(500);
  //demo();
#if defined(__AVR__) || defined(ESP8266)
  motionloop();
  if (ack_waiting) {
    xprintf(PSTR("ok\n"));
    ack_waiting = 0;
  }
  if (Serial.available() > 0)
  {
    line_done = gcode_parse_char(Serial.read());
    if (line_done) {
      ack_waiting = line_done - 1;
    }
  }
#else
#endif

}
void loop() {
  demo();
}
