
#include "common.h"
#include "gcode.h"
#include "timer.h"
#include<stdint.h>
extern void demo();
extern void motionloop();
void setup() {
  // put your setup code here, to run once:
//  Serial.setDebugOutput(true);
  Serial.begin(115200);
  initmotion();
  init_gcode();
  zprintf(PSTR("start\nok\n"));
  zprintf(PSTR("Motion demo\nok\n"));
}


int line_done, ack_waiting = 0;
void gcode_loop() {
  //float x=12.345;
  //xprintf(PSTR("Motion demo %d %f\n"),10,x);
  //delay(500);
  //demo();
#if defined(__AVR__) || defined(ESP8266)
  motionloop();
  if (ack_waiting) {
    zprintf(PSTR("ok\n"));
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
  //demo();
  gcode_loop();
/*  if (feedthedog()){
    float f=123.456;
    int32_t i=1234;
    xprintf("%f",ff(f),i);
  }  
*/
}
