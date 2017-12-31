#include "motion.h"
#include "common.h"
void setup() {
  // put your setup code here, to run once:
initmotion();
//Serial.setDebugOutput(true);
 Serial.begin(115200);
}
void demo(){
  addmove(30,10,50,0);
  addmove(30,-20,50,0);
  addmove(30,10,50,0);
  startmove();
  waitbufferempty();

}
void loop() {
  double x=12.345;
  xprintf(PSTR("Motion demo %d %f\n"),10,x);
  delay(2000);
  demo();
}
