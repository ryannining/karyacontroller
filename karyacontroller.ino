#include "motion.h"
#include "common.h"
void setup() {
  // put your setup code here, to run once:
initmotion();
//Serial.setDebugOutput(true);
 Serial.begin(115200);
}
void demo(){
  addmove(30,100,50,0);
  addmove(30,-200,50,0);
  addmove(30,100,50,0);
  startmove();
  waitbufferempty();

}
void loop() {
  delay(2000);
  xprintf(PSTR("Motion demo\n"));
  demo();
}
