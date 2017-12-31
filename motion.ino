#include "motion.h"
void setup() {
  // put your setup code here, to run once:
initmotion();
//Serial.setDebugOutput(true);
Serial.begin(115200);
}
void demo(){
  addmove(30,300,50,0);
  addmove(30,500,100,0);
  addmove(30,100,150,0);
  startmove();
  waitbufferempty();

}
void loop() {
  delay(3000);
#if defined(__AVR__)
// AVR specific code here
Serial.write("AVR\n");
#elif defined(ESP8266)
Serial.write("ESP8266\n");
#else
  
  Serial.write("Other\n");
#endif
  demo();
}
