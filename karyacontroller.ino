//#define timing
//#define timingG
//#define echoserial

//#define VERSION "Firmware 1.0.0" // first to have firmware update
#define VERSION "Firmware 1.0.1" // new remote, able to control more on remote
#define VERSION "Firmware 1.0.2" // fix lots issues with remote, LCD
#define VERSION "Firmware 1.0.3" // fix lots issues with remote, LCD, PLASMA THC,Better Loop
#define VERSION "Firmware 1.0.4" // fix pause, stop, smooth stop on pause/stop, reduce use of waitbufferempty
#define VERSION "Firmware 1.0.5" // new feature: resume job
#define VERSION "Firmware 1.1.0" // 1 firmware for all machine, config.ini based configuration
#define VERSION "Firmware 1.1.1" // able to change laser to router press Inp on remote, able
// to change laser on on Low or High
// calibrate on LCD
//
#define VERSION "Firmware 1.1.2" // change to fixed rate timer
#define VERSION "Firmware 1.2" // reduce lots of code, remove gcode processing, integrate karyalcd and new code irremote

#define VERSION "Firmware 1.3.0" // Focus on esp32+esp8266, Input Shaping, Single Timer for motor and PWM

#include <WiFiClient.h>
#define WIFI_AUTH_OPEN ENC_TYPE_NONE
#ifdef ESP8266
#include <FS.h> // Include the LittleFS library
#define USE_CLIENTSSL false
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>

ESP8266WebServer server(80);

// ===============================
#else
#include <WiFi.h>
#include <WiFiAP.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Update.h>
WebServer server(80);
#endif


//#define USE_CLIENTSSL true
#include "shaper.h"
#include "common.h"

#include "gcode.h"
#include "temp.h"
#include "timer.h"
#include "eprom.h"
#include "gcodesave.h"
#include "karyalcd.h"
#include "ir_remote.h"
#include "ir_oled.h"

#include <stdint.h>



#ifdef USEOTA
#include <ArduinoOTA.h>
#endif




// ==============================






WiFiClient client;



String bot_token;
String report_id;

#include "UrlEncode.h"
IPAddress serverIP;
String karyacncIP;
int connect_timeout = 10;
IPAddress fixedip;

extern float stepmmy;
extern int steppiny, dirpiny;

extern bool isRotary;
void sendTelegram(String stext) {
  int ll = karyacncIP.length();
  Serial.print("T>");Serial.println(stext);
  if (ll || serverIP[0] > 1) {
    HTTPClient http;
    String Link = "http://";
    if (ll)Link += karyacncIP;
    else Link += String(serverIP[0]) + "." + String(serverIP[1]) + "." + String(serverIP[2]) + "." + String(serverIP[3]);
    Link += ":8888/t?bot=" + bot_token + "&id=" + report_id + "&t=" + urlEncode(stext);
    http.begin(client, Link);
    int httpCode = http.GET();
    http.end();
  }
}
void downloadFile(String durl, String outf) {
}
bool canuseSerial=false;

int strToPin(File f1, char u = '\n') {
    String s2 = f1.readStringUntil(u);
  #ifdef ESP32
  if (s2 == "D1")return 22;
  if (s2 == "D2")return 21;
  if (s2 == "D3")return 17;
  if (s2 == "D4")return 16;
  if (s2 == "D5")return 18;
  if (s2 == "D6")return 19;
  if (s2 == "D7")return 23;
  if (s2 == "D8")return 5;
  if (s2 == "D0")return 26;
  if (s2 == "TX")return TX;
  if (s2 == "RX")return RX;  
  if (s2 == "A0")return 36;
  if (s2 == "25")return 25;
  if (s2 == "26")return 26;
  if (s2 == "27")return 27;
  if (s2 == "22")return 22;
  if (s2 == "23")return 23;

  #else


  if (s2 == "D1")return D1;
  if (s2 == "D2")return D2;
  if (s2 == "D3")return D3;
  if (s2 == "D4")return D4;
  if (s2 == "D5")return D5;
  if (s2 == "D6")return D6;
  if (s2 == "D7")return D7;
  if (s2 == "D8")return D8;
  if (s2 == "D0")return D0;
  if (s2 == "TX")return TX;
  if (s2 == "RX")return RX;  
  if (s2 == "A0")return A0;
  if (s2 == "?")return 255;
  #endif
  return -1;
}
int strToMt(String s2) {
  if (s2 == "X")return 0;
  if (s2 == "Y")return 1;
  if (s2 == "Z")return 2;
  if (s2 == "R")return 3;
  return 0;
}

float constrainF(float a, float b, float c) {
  if (a < b)a = b;
  if (a > c)a = c;
  return a;
}
float constrainF1(float a, float b, float c) {
  float a1 = fabs(a);
  if (a1 < b)a1 = b;
  if (a1 > c)a1 = c;
  return a < 0 ? -a1 : a1;
}
int constrainL(int a, int b, int c) {
  if (a < b)a = b;
  if (a > c)a = c;
  return a;
}

bool makezeropoint;
bool allowxyjog;
extern int zerocrosspin;
extern float f2_multiplier;

void readConfig(String fn) {
  int vv = 0;

  if (!SPIFFS.exists(fn))return;
  File f1;
  f1 = SPIFFS.open(fn, "r");
  if (!f1) {
    //zprintf(PSTR("File Config found\n"));
    return;
  }
  String s1, s2, s3, s4, s5, s6;

  while  (f1.available()) {
    s1 = f1.readStringUntil('=');
    if (s1 == ";") {
      s1 = f1.readStringUntil('\n');
    } else if (s1 == "connect_timeout") {
      connect_timeout = constrainL(f1.readStringUntil('\n').toInt(), 0, 300);
    } else if (s1 == "testlaser") {
      extern int testlaserdur;
      testlaserdur = 3000 + constrainL(f1.readStringUntil('\n').toInt(), 0, 5000);
    } else if (s1 == "cutpause") {
      extern int cutpause;
      cutpause = constrainL(f1.readStringUntil('\n').toInt(), 0, 10000);
    } else if (s1 == "stepdelay") {
      extern int stepdelay;
      stepdelay = constrainL(f1.readStringUntil('\n').toInt(), 0, 1000);
    } else if (s1 == "fixed_ip") {
      if (!fixedip.fromString(f1.readStringUntil('\n')))fixedip[0] = 0;
    } else if (s1 == "SSID") {
      wifi_ap = f1.readStringUntil('\n'); wifi_ap.trim();
    } else if (s1 == "PASSWORD") {
      wifi_pwd = f1.readStringUntil('\n'); wifi_pwd.trim();
    } else if (s1 == "machine_name") {
      wifi_dns = f1.readStringUntil('\n'); wifi_dns.trim();
    } else if (s1 == "motor") {
      // no,step/mm,speed
      s2 = f1.readStringUntil(',');
      int mt = strToMt(s2);
      stepmmx[mt] = constrainF1(String(f1.readStringUntil(',')).toFloat(), 20, 5000); // stepmmx
      maxf[mt] = constrainF(String(f1.readStringUntil(',')).toFloat(), 1, 300); // maxf
      maxa[mt] = constrainF(String(f1.readStringUntil(',')).toFloat(), 10, 10000); // maxa
      xback[mt] = constrainF(String(f1.readStringUntil('\n')).toFloat(), -2, 2); // xback
      //if (maxa[mt]<110)maxa[mt]=110;
      //if (maxf[mt]<5)maxa[mt]=5;
    } else if (s1 == "motor_pin") {
      s2 = f1.readStringUntil(',');
      int mt = strToMt(s2);
      mdir_pin[mt] = strToPin(f1, ',');
      mstep_pin[mt] = strToPin(f1);
    } else if (s1 == "home") {
      ax_home[0] = String(f1.readStringUntil(',')).toFloat(); // maxf
      ax_home[1] = String(f1.readStringUntil(',')).toFloat(); // maxa
      ax_home[2] = String(f1.readStringUntil('\n')).toFloat(); // xback
    } else if (s1 == "home_ofs") {
      axisofs[0] = String(f1.readStringUntil(',')).toFloat(); // maxf
      axisofs[1] = String(f1.readStringUntil(',')).toFloat(); // maxa
      axisofs[2] = String(f1.readStringUntil('\n')).toFloat(); // xback
    } else if (s1 == "home_feed") {
      homingspeed = f1.readStringUntil('\n').toFloat();
    } else if (s1 == "bot_token") {
      bot_token = f1.readStringUntil('\n');
    } else if (s1 == "master") {
      karyacncIP = f1.readStringUntil('\n');
    } else if (s1 == "report_id") {
      report_id = f1.readStringUntil('\n');
    } else if (s1 == "limit_pin") {
      limit_pin = strToPin(f1);
    } else if (s1 == "zerocross_pin") {
      zerocrosspin = strToPin(f1);
    } else if (s1 == "mode") {
      s2 = f1.readStringUntil('\n');
      if (s2 == "router")lasermode = 0;
      if (s2 == "laser")lasermode = 1;
      if (s2 == "plasma")lasermode = 2;
      vv = 1;
    } else if (s1 == "pwm_pow") {
      extern float pwm_pow;
      pwm_pow = f1.readStringUntil('\n').toFloat();
    } else if (s1 == "lcd_addr") {
      lcd_rst = f1.readStringUntil('\n').toInt();
    } else if (s1 == "ir_pin") {
      lIR_KEY = strToPin(f1);
    } else if (s1 == "lcd_rst") {
      lcd_rst = strToPin(f1);
    } else if (s1 == "lcd_sda") {
      lcd_sda = strToPin(f1);
    } else if (s1 == "lcd_sda2") {
      lcd_sda2 = strToPin(f1);
    } else if (s1 == "lcd_contrast") {
      lcd_contrast = constrainF(f1.readStringUntil('\n').toFloat(), 1, 15);
    } else if (s1 == "lcd_scl") {
      lcd_scl = strToPin(f1);
    }  else if (s1 == "lcd_cs") {
      // dont care about CS for now
      s2 = f1.readStringUntil('\n');
    }  else if (s1 == "lcd_type") {
      s2 = f1.readStringUntil('\n');
      // default
      lcd_kind = KIND_NK1661;
      //if (s2=="NK1661")lcd_kind=KIND_NK1661;else
      if (s2 == "NK1202")lcd_kind = KIND_NK1202; else if (s2 == "NK6100")lcd_kind = KIND_NK6100; else if (s2 == "OLED1306")lcd_kind = KIND_OLED1306; else if (s2 == "LCD2004")lcd_kind = KIND_LCD2004; else if (s2 == "ST7565")lcd_kind = KIND_ST7565; else if (s2 == "ST7735")lcd_kind = KIND_ST7735;
    }  else if (s1 == "tool_pin") {
      atool_pin = strToPin(f1);
    }  else if (s1 == "pwm_pin") {
      pwm_pin = strToPin(f1);
    } else if (s1 == "corner") {
      xycorner = constrainL(f1.readStringUntil('\n').toInt(), 5, 100);
    } else if (s1 == "min_pwm_clock") {
      extern int min_pwm_clock;
      // in uS
      min_pwm_clock = constrainF(5 * f1.readStringUntil('\n').toFloat(), 0, 1000);
    } else if (s1 == "lscale") {
      Lscale = constrainF(f1.readStringUntil('\n').toFloat(), 0.1, 50);
    } else if (s1 == "fscale") {
      f2_multiplier = constrainF(f1.readStringUntil('\n').toFloat(), 0.1, 2);
    } else if (s1 == "skew_y") {
      skew_y = constrainF(f1.readStringUntil('\n').toFloat(), -50, 50);
    } else if (s1 == "thc_up") {
      thc_up = f1.readStringUntil('\n').toFloat();
    } else if (s1 == "thc_ofs") {
      thc_ofs = f1.readStringUntil('\n').toFloat();
    } else if (s1 == "allowxyjog") {
      allowxyjog = f1.readStringUntil('\n') == "Y";
    } else if (s1 == "makezeropoint") {
      makezeropoint = f1.readStringUntil('\n') == "Y";
    } else if (s1 == "laser_on") {
      extern bool TOOLONS[3];
      TOOLONS[1] = f1.readStringUntil('\n') == "H" ? HIGH : LOW;
    } else if (s1 == "plasma_on") {
      extern bool TOOLONS[3];
      TOOLONS[2] = f1.readStringUntil('\n') == "H" ? HIGH : LOW;
    } else if (s1 == "trimmer_on") {
      extern bool TOOLONS[3];
      TOOLONS[0] = f1.readStringUntil('\n') == "H" ? HIGH : LOW;
    } else if (s1 == "showkey") {
      extern bool showremotekey;
      showremotekey = f1.readStringUntil('\n') == "Y";
    } else if (s1 == "water") {
      if (f1.readStringUntil('\n') == "Y") {
        water_pin = A0;
        //pinMode(A0,INPUT_PULLDOWN);
      } else water_pin = -1;
    } else if (s1 == "thc") {
      thc_enable = f1.readStringUntil('\n') == "Y";
    } else if (s1 == "temp_pin") {
      ltemp_pin = strToPin(f1);
    } else if (s1 == "temp_limit") {
      temp_limit = constrainL(f1.readStringUntil('\n').toInt(), 30, 80);
    } else if (s1 == "buzzer") {
      BUZZER_ERR = strToPin(f1);
      pinmode(BUZZER_ERR, OUTPUT);
    } else if (s1 == "input_shaping") {
        s2 = f1.readStringUntil(',');
        float freq = String(f1.readStringUntil(',')).toFloat();
        float damp = String(f1.readStringUntil('\n')).toFloat();
        uint8_t type = SHAPER_NONE;
        if (s2 == "ZV") type = SHAPER_ZV;
        else if (s2 == "ZVD") type = SHAPER_ZVD;
        else if (s2 == "MZV") type = SHAPER_MZV;
        else if (s2 == "EI") type = SHAPER_EI;

        shaper.configure(freq, damp, type);
    } else {
      f1.readStringUntil('\n');
    }
  }
  f1.close();
  if (vv == 0) {
    if (fn == "/config.ini")SPIFFS.remove(fn);
  }
}
void printmotor(File f1, int mt) {
  f1.print(stepmmx[mt], 3); f1.print(",");
  f1.print(maxf[mt]); f1.print(",");
  f1.print(maxa[mt]); f1.print(",");
  f1.println(xback[mt], 3);
}
void saveconfigs() {
  File f1;
  f1 = SPIFFS.open("/custom.ini", "w");
  f1.print("SSID="); f1.println(wifi_ap);
  f1.print("PASSWORD="); f1.println(wifi_pwd);
  f1.print("machine_name="); f1.println(wifi_dns);
  f1.print("motor=X,"); printmotor(f1, 0);
  f1.print("motor=Y,"); printmotor(f1, 1);
  f1.print("motor=Z,"); printmotor(f1, 2);
  f1.print("motor=R,"); printmotor(f1, 3);
  f1.print("corner="); f1.println(xycorner);
  f1.print("lscale="); f1.println(1.0/Lscale);
  f1.print("skew_y="); f1.println(skew_y);
  f1.print("thc_up="); f1.println(thc_up);
  f1.print("thc_ofs="); f1.println(thc_ofs);
  f1.print("home="); f1.print(ax_home[0], 2); f1.print(",");
  f1.print(ax_home[1], 2); f1.print(",");
  f1.println(ax_home[2], 2);
  f1.print("home_ofs="); f1.print(axisofs[0], 2); f1.print(",");
  f1.print(axisofs[1], 2); f1.print(",");
  f1.println(axisofs[2], 2);
  f1.close();
  pre_motion_set();
}
void readconfigs() {
  //noInterrupts();
  canuseSerial=true;
  f2_multiplier = 1;
  mstep_pin[2] = -1;
  mdir_pin[2] = -1;
  skew_y = 0;
  fixedip[0] = 0;
  extern float pwm_pow;
  pwm_pow = 1;

  karyacncIP = "";
  wifi_ap = "";
  lcd_sda2 = 255;
  lcd_scl = 255;

  pwm_pin = -1;

  TOOLONS[0] = HIGH;
  TOOLONS[1] = HIGH;
  TOOLONS[2] = HIGH;

  zerocrosspin = -1;
  /*
  lcd_kind = KIND_NK1661;
  lcd_sda = TX;
  lcd_scl = RX;
  lIR_KEY = TX;
  lcd_rst = D1;
  */
  //allow serial debugging
  #ifdef ESP8266
  lcd_kind = KIND_ST7565;
  lcd_sda = TX;
  lcd_sda2 = D1;
  lcd_scl = RX;
  lIR_KEY = TX;
  lcd_rst = TX;
  #else
  lcd_kind = KIND_ST7565;
  lcd_sda = 26;
  lcd_sda2 = 25;
  lcd_scl = 27;
  lIR_KEY = 26;
  lcd_rst = 26;
  #endif

  makezeropoint = true;
  allowxyjog = false;

  Lscale = 1;
  xycorner = 20;
  shaper.configure(50,0.1,SHAPER_MZV);
  readConfig("/config.ini");
  readConfig("/custom.ini");
  // calculate something
  TOOLON = TOOLONS[lasermode];
  pre_motion_set();
  restartLCD();
  init_temp();
  if (pwm_pin == -1)pwm_pin = atool_pin;
  pinmode(atool_pin, OUTPUT);
  pinmode(pwm_pin, OUTPUT);
  dwrite(atool_pin, !TOOLON);
  dwrite(pwm_pin, !TOOLON);


  stepmmy = stepmmx[1];
  dirpiny = mdir_pin[1];
  steppiny = mstep_pin[1];
  isRotary = false;
  // Set the Telegram bot properies
  //myBot.token=Buf;
  //myBot.sendMessage(report_id, "test");
  //interrupts();

  canuseSerial=!(lcd_sda==TX || lcd_scl==RX);
  //canuseSerial=true;
  if (canuseSerial){
    // pinMode(RX, INPUT);
    // pinMode(TX, OUTPUT);
    Serial.begin(115200);
    Serial.println("Log:");
    //Serial.setDebugOutput(true);
  }
   else Serial.end();
   Lscale=1.0/Lscale;

}
IPAddress ip;
// Server
#define IOT_IP_ADDRESS "172.245.97.171"
long lm = 0;
int ISWIFIOK = 0;

File fsUploadFile;

#define NOINTS
// noInterrupts();
#define INTS
// interrupts();

String getContentType(String filename)
{ // convert the file extension to the MIME type
  if (filename.endsWith(".html"))
    return "text/html";
  else if (filename.endsWith(".css"))
    return "text/css";
  else if (filename.endsWith(".js"))
    return "application/javascript";
  else if (filename.endsWith(".ico"))
    return "image/x-icon";
  else if (filename.endsWith(".gz"))
    return "application/x-gzip";
  return "text/plain";
}
bool handleFileRead(String path)
{ // send the right file to the client (if it exists)

  //NOINTS
  if (SPIFFS.exists(path + ".html"))
    path += ".html";
  else if (path == "/config")
  {
    path = "/karyaconfig.html"; // If a folder is requested, send the index file
    if (!SPIFFS.exists(path)) {
      server.send(200, "text/html", web_config);
      return true;
    }
  }
  else if (path == "/karyacnc.7z" || path == "/karyacnc.zip")
  {
#ifdef karyacnc_zip_len
    server.sendHeader("Content-Disposition", "attachment; filename=karyacnc.7z");
    server.send(200, "application/x-zip", karyacnc_zip, karyacnc_zip_len);
    return true;
#endif
  }
  else if (path == "/config.ini")
  {
    if (!SPIFFS.exists(path)) {
      server.send(200, "text/plain", web_configini);
      return true;
    }
  }
  else if (path == "/--gcodex.js")
  {
    if (!SPIFFS.exists(path)) {
      server.send(200, "application/javascript", web_gcodex);
      return true;
    }
  }  else if (path == "/websocket.js")
  {
    if (!SPIFFS.exists(path)) {
      server.send(200, "application/javascript", web_websocket);
      return true;
    }
  }
  else if (path.endsWith("/cnc"))
  {
    path = "/cnc.html"; // If a folder is requested, send the index file
    if (!SPIFFS.exists(path)) {
      server.send(200, "text/html", web_cnc);
      return true;
    }
  }
  else if (path.endsWith("/")) {
    path = "/jobs.html"; // If a folder is requested, send the index file
    if (!SPIFFS.exists(path)) {
      server.send(200, "text/html", web_index);
      return true;
    }
  }


  //xprintf(PSTR("handleFileRead: %s\n"), path.c_str());
  String contentType = getContentType(path); // Get the MIME type
  String pathWithGz = path + ".gz";
  bool gz = SPIFFS.exists(pathWithGz);
  if (gz || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    File file = SPIFFS.open(gz ? pathWithGz : path, "r"); // Open the file
    size_t sent = server.streamFile(file, contentType); // Send it to the client
    file.close(); // Close the file again
    //xprintf(PSTR("\tSent file: %s\n"), path.c_str());
    return true;
  }
  //else
  //xprintf(PSTR("\tFile Not Found: %s\n"), path.c_str()); // If the file doesn't exist, return false

  //INTS
  return false;
}

void updateFirmware()
{
  File file = SPIFFS.open("/firmware.bin", "r");
  if (!file) {
    d_clear();
    d_text(0, 0, "/firmware.bin");
    d_text(0, 1, "Not found");
    d_show();
    delay(2000);
    return;
  }
  size_t fileSize = file.size();
  if (!Update.begin(fileSize)) {
    d_clear();
    d_text(0, 0, "Update failed !");
    d_text(0, 1, "Not enough space");
    d_show();
    delay(2000);
    return;
  }
  d_clear();
  d_text(0, 0, "Update Firmware");
  d_text(0, 1, "Please wait");
  d_show();
  Update.writeStream(file);
  if (Update.end()) {
    file.close();
    SPIFFS.remove("/firmware.bin");
    d_clear();
    d_text(0, 0, "Update Firmware");
    d_text(0, 1, "Success !");
    d_show();
    delay(3000);
    ESP.restart();
  }
  else {
    d_clear();
    d_text(0, 0, "Failed.");
    d_text(0, 1, "Try again...");
    d_show();
    file.close();
    delay(3000);
  }
}

bool isconfig, isfirmware;
void handleFileUpload()
{ // upload a new file to the LittleFS
  NOINTS
  if (uncompress) {
    server.send(500, "text/plain", "Still PRINTING");
  }
  else {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (filename == "")
        filename = "/gcode.gcode";
      if (!filename.startsWith("/"))
        filename = "/" + filename;
      // always rename any config to config.ini, except custom.ini
      isconfig = filename.endsWith(".ini");
      isfirmware = filename.endsWith(".bin");
      if (isconfig && !filename.startsWith("/custom"))filename = "/config.ini";
      if (isfirmware)filename = "/firmware.bin";
      zprintf(PSTR("handleFileUpload Name: %s\n"), filename.c_str());
      fsUploadFile = SPIFFS.open(filename, "w"); // Open the file for writing in LittleFS (create if it doesn't exist)
      //if (!fsUploadFile)zprintf(PSTR("Error file\n"));
      //filename = String();
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
      // do temploop to avoid overheating
      //temp_loop(micros());
      if (fsUploadFile)
        fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
    }
    else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) { // If the file was successfully created
        fsUploadFile.close(); // Close the file again
        xprintf(PSTR("handleFileUpload Size: %d\n"), upload.totalSize);
        // server.sendHeader("Location", "/upload"); // Redirect the client to the success page
        // server.send(303);
        if (isconfig)readconfigs();
      }
      else {
        server.send(500, "text/plain", "500: couldn't create file");
      }
    }
  }
  INTS
}




void connectWifi(int ret = 1)
{
  if (ret) {
    WiFi.reconnect();
    return;
  }

  xprintf(PSTR("Try connect wifi AP:%s \n"), wifi_ap);
  WiFi.mode(WIFI_STA);
  if (fixedip[0] > 0) {
    IPAddress gateway;
    gateway = fixedip;
    gateway[3] = 1;
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(fixedip, gateway, subnet);
  }
  WiFi.begin(wifi_ap, wifi_pwd);

  int cntr = 10;//connect_timeout;
  WiFi.waitForConnectResult();

  while (WiFi.status() != WL_CONNECTED && cntr > 0) {
    delay(200);

    d_clear();
    d_text(0, 0, VERSION);
    d_text(0, 1, "connecting ... " + String(cntr));
    d_text(0, 2, wifi_ap);
    d_show();

    cntr--;
    //xprintf(PSTR("."));
    Serial.print(".");
  }

  //WiFi.setAutoReconnect(true);
  //WiFi.persistent(true);

}
bool isSTA;
void reset_factory() {
  
  SPIFFS.remove("/custom.ini");
  readconfigs();
}

String IPA;
void setupwifi(int num)
{
  NOINTS

  while (1) {
    //xprintf(PSTR("Wifi Initialization\n"));
    if (num) {
      //xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );
      //xprintf(PSTR("Disconnect\n"));
      WiFi.disconnect();
      server.close();
      //servertcp.close();

    }
    if (wifi_ap) connectWifi(0);
    String ipa = "->" + wifi_ap;
    ISWIFIOK = 0;
    if ((connect_timeout > 0 && WiFi.status() != WL_CONNECTED) || wifi_ap.length() == 0) {
      xprintf(PSTR("Access Point Mode\n"));
      /*IPAddress local_IP(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 9);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_IP, gateway, subnet);
      */
      WiFi.mode(WIFI_AP);
      ISWIFIOK = 0;

      if (wifi_dns == "")wifi_dns = "ESP_CNC";
      WiFi.softAP(wifi_dns);

      //ip = WiFi.softAPIP();
      //xprintf(PSTR("AP:%s Ip:%d.%d.%d.%d\n"), ((' '<wifi_dns[0]<'Z')?wifi_dns:ssid), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]));
      ipa = wifi_dns + " : 4.1";
    }
    IPA = ipa;
    wifi_loop();




   
    server.on("/setconfig", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      wifi_dns = server.arg("name");
      wifi_ap = server.arg("ap");
      wifi_pwd = server.arg("pw");
      server.send(200, "text/html", "OK");
      saveconfigs();
      delay(1000);
      ESP.restart();
      INTS
    });

    server.on("/gcode", HTTP_GET, []() { // if the client requests the upload page

      server.send(200, "text/html", "Doesnot support GCODE anymore");

    });
    server.on("/speed", HTTP_GET, []() { // if the client requests the upload page
      extern uint8_t head, tail, tailok;
      char gc[10];
      server.arg("t").toCharArray(gc, 9);
      extern float tf_multiplier;
      tf_multiplier = atoi(gc) * 0.01;
      tailok = tail + 5; // need to replanning all moves
      server.send(200, "text/html", "OK");

    });
    server.on("/tool", HTTP_GET, []() { // if the client requests the upload page
      int v = server.arg("t").toInt();
      extern void testLaser(void);
      if (v == 125)testLaser(); else set_tool(v);
      server.send(200, "text/html", "OK");
    });
    server.on("/updatefirmware", HTTP_GET, []() { // if the client requests the upload page
      server.send(200, "text/html", "OK");
      updateFirmware();
    });

    server.on("/getconfig", HTTP_GET, []() { // if the client requests the upload page
      String st = "disconnected";
      if (WiFi.status() == WL_CONNECTED)
        st = "connected";
      server.send(200, "text/html", "['" + wifi_ap + "','" + wifi_pwd + "','" + wifi_dns + "','" + st + "']");
    });
    server.on("/scanwifi", HTTP_GET, []() {
      int n = WiFi.scanNetworks();
      String res = "[";
      if (n == 0) {
      }
      else {
        for (int i = 0; i < n; ++i) {
          // Print SSID and RSSI for each network found
          if (i)
            res += ",";
          res += "['" + WiFi.SSID(i) + "','" + WiFi.RSSI(i) + "',";
          if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN)
            res += "''";
          else
            res += "'*'";
          res += "]";
        }
      }
      res += "]";
      server.send(200, "text/html", res);
    });
#ifdef ANALOG_THC
    server.on("/thc", HTTP_GET, []() {
      extern String formatthc();
      String res = formatthc();
      server.send(200, "text/html", res);
    });
#endif
#ifdef PLOTTING
    server.on("/temp", HTTP_GET, []() {
      extern String formattemp();
      String res = formattemp();
      server.send(200, "text/html", res);
    });
    server.on("/velo", HTTP_GET, []() {
      extern String formatvelo();
      String res = formatvelo();
      server.send(200, "text/html", res);
    });
#endif
    server.on("/pauseprint", HTTP_GET, []() { // if the client requests the upload page
      //NOINTS
      if (uncompress) {
        ispause = ispause == 0 ? 1 : 0;
        extern int8_t PAUSE;
        PAUSE = ispause;
        server.send(200, "text/html", ispause == 1 ? "PAUSED" : "RESUMED");
      }
      else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/probe", HTTP_GET, []() {
      //NOINTS
      if (!uncompress) {
        docheckendstop(1);
        extern int16_t endstopstatus;
        server.send(200, "text/html", endstopstatus < 0 ? "HIGH" : "LOW");
      }
      else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/resumeprint", HTTP_GET, []() { // if the client requests the upload page
      //NOINTS
      if (uncompress) {
        ispause = 0;
        extern int8_t PAUSE;
        PAUSE = ispause;
        server.send(200, "text/html", "OK");
      }
      else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/startprint", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send(200, "text/html", "FAIL, STILL PRINTING");
      }
      else {
        server.send(200, "text/html", "OK");
        beginuncompress("/gcode.gcode", false, 0);
      }
      INTS
    });
    server.on("/startjob", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send(200, "text/html", "FAIL, STILL PRINTING");
      }
      else {
        extern String jobname;
        jobname = server.arg("jobname");
        beginuncompress(jobname, false, 0);
        server.send(200, "text/html", "Start Ok");
      }
      INTS
    });

    server.on("/previewjob", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send(200, "text/html", "FAIL, STILL PRINTING");
      }
      else {
        extern String jobname;
        jobname = server.arg("jobname");
        extern void check_job();
        extern void runpreview();
        check_job();
        runpreview();
      }
      INTS
    });
    server.on("/removejob", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send(200, "text/html", "FAIL, STILL PRINTING");
      }
      else {
        deletejob(server.arg("jobname"));
        server.send(200, "text/html", "Delete Ok");
      }
      INTS
    });
    server.on("/renamejob", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send(200, "text/html", "FAIL, STILL PRINTING");
      }
      else {
        SPIFFS.rename(server.arg("jobname"), server.arg("newname"));
        SPIFFS.rename(server.arg("jobname") + ".jpg", server.arg("newname") + ".jpg");
        server.send(200, "text/html", "Rename Ok");
      }
      INTS
    });
    server.on("/getfree", HTTP_GET, []() {
      if (!uncompress) {
        #ifdef ESP32
        // FSInfo fs_info;
        // SPIFFS.info(fs_info);
        int tBytes = SPIFFS.totalBytes();
        int uBytes = SPIFFS.usedBytes();
        #else
        FSInfo fs_info;
        SPIFFS.info(fs_info);
        int tBytes = fs_info.totalBytes;
        int uBytes = fs_info.usedBytes;
        #endif
        server.send(200, "text/plain", "[" + String(tBytes) + "," + String(uBytes) + "]");
      }
    });
    server.on("/clearjobs", HTTP_GET, []() {
      #ifdef ESP32
      File dir = SPIFFS.open("/");
      File file;
      while (file=dir.openNextFile()) {
        if (!file.isDirectory()){
          String s = file.name();
      #else
      Dir dir = SPIFFS.openDir("/");
        while (dir.next()) {
          String s = dir.fileName();
          {
      #endif
          if (s.endsWith(".gcode")|| s.endsWith(".jpg")) {
            SPIFFS.remove(s);
          }
        }
      }
      server.send(200, "text/plain", "Ok");
      
    });
    server.on("/getjobs", HTTP_GET, []() {
      NOINTS
      if (!uncompress) {
        String str = "[";
#ifdef ESP32
        File dir = SPIFFS.open("/");
        File file = dir.openNextFile();
        while (file) {
          String s = file.name();
          if (s.endsWith(".gcode")) {
            if (str.length() > 2)
              str += ",";
            str += "['/";
            str += file.name();
            str += "',";
            str += file.size();
            str += "]";
          }
          file = dir.openNextFile();
        }
#elif ESP8266
        Dir dir = SPIFFS.openDir("/");
        while (dir.next()) {
          String s = dir.fileName();
          if (s.endsWith(".gcode")) {
            if (str.length() > 2)str += ",";
            str += "['";
            str += dir.fileName();
            str += "',";
            str += dir.fileSize();
            str += "]";
          }
        }
#endif

        str += "]";
        server.send(200, "text/plain", str);
      }
      else {
        server.send(200, "text/plain", "ERR");
      }
      INTS
    });
    server.on("/stopprint", HTTP_GET, []() { // if the client requests the upload page
      if (!uncompress) {
        server.send(200, "text/html", "FAIL, NOT PRINTING");
      }
      else {
        server.send(200, "text/html", "OK");
        extern void stopmachine();
        stopmachine();
      }
    });
    server.on("/home", HTTP_GET, []() { // if the client requests the upload page
      addmove(100, 0, 0, 10, 0, 1, 1);
      addmove(100, 0, 0, 10, 0, 1, 0);
      addmove(100, 0, 0, 0, 0, 1, 0);
      server.send(200, "text/html", "OK");
      //homing();
    });
    server.on("/jogmove", HTTP_GET, []() { // if the client requests the upload page
      server.send(200, "text/html", "OK");
      if (!uncompress) {
        float x, y, z;
        x = server.arg("x").toFloat();
        y = server.arg("y").toFloat();
        z = server.arg("z").toFloat();
        addmove(100, x, y, z, 0, 1, 1);
      }
    });


    server.on("/t", HTTP_GET, []() { // if the client requests the upload page
      serverIP = server.client().remoteIP();
      server.send(200, "text/plain", "OK");
      sendTelegram("test");
    });

    server.on("/upload", HTTP_GET, []() { // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      serverIP = server.client().remoteIP();
      //serverIP=String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);

      server.send(200, "text/html", "<form method=\"post\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"name\"> <input class=\"button\" type=\"submit\" value=\"Upload\"></form>");
    });
    server.on("/delete", HTTP_GET, []() { // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      String s = server.arg("fn");
      SPIFFS.remove(s);
      server.send(200, "text/html", "Delete " + server.arg("fn"));
      if (s.endsWith(".ini"))readconfigs();
    });
    server.on("/rename", HTTP_GET, []() { // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      SPIFFS.rename(server.arg("fn"), server.arg("fnew"));
      server.send(200, "text/html", "Rename to " + server.arg("fnew"));
    });

    server.on("/upload", HTTP_POST, // if the client posts to the upload page
    []() {
      // server.sendHeader("Access-Control-Expose-Headers", "Access-Control-*");
      // server.sendHeader("Access-Control-Allow-Headers", "Access-Control-*, Origin, X-Requested-With, Content-Type, Accept");
      // server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS, HEAD");
      // server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200, "text/plain", "");
    }, // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload // Receive and save the file
             );

    server.onNotFound([]() { // If the client requests any URI
      if (!handleFileRead(server.uri())) // send it if it exists
        server.send(404, "text/plain", "404: Not Found ?"); // otherwise, respond with a 404 (Not Found) error
    });


    //if (MDNS.begin(wifi_ap)) {
    //Serial.println("MDNS responder started");
    //}

#ifdef TCPSERVER
    servertcp.begin();
#endif


    if (num)
      return;



    if (ip[2] == 1) {
      //downloadFile("http://192.168.1.13:8888/esp/websocket.js","/websocket.js");
      //downloadFile("http://192.168.1.13:8888/esp/jobs.html","/jobs.html");
      //downloadFile("http://192.168.1.13:8888/esp/cnc.html","/cnc.html");
      //downloadFile("http://192.168.1.13:8888/esp/karyaconfig.html","/karyaconfig.html");
      //downloadFile("http://192.168.1.13:8888/esp/test.gcode","/test.gcode");
      //downloadFile("http://192.168.1.13:8888/esp/config.ini","/config.ini");
    }


#ifdef USEOTA
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // LittleFS
        type = "filesystem";

      // NOTE: if updating LittleFS this would be the place to unmount LittleFS using LittleFS.end()
      //Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      //Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      /*Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      */
    });
    ArduinoOTA.begin();
#endif
    break;
  }
  //  return;

  INTS
}
extern int sendwait;
boolean alreadyConnected = false;
uint32_t wmc = 0;
extern uint32_t cm;

uint32_t i_l_c = 0;

void important_loop() {
#ifdef ESP32  
  if (cm - wmc < (uncompress ? 150000 : 30000)) {
#else
  if (cm - wmc < (uncompress ? 500000 : 100000)) {
#endif
    return;
  }
  IR_loop(0);
  //server.handleClient();
  /*
    motionloop();
    motionloop();

  */
  wmc = cm;
}
long ipcheck = 0;
void wifi_loop()
{
  extern int nextok;

  // if motor idle
   if ( WiFi.status() == WL_CONNECTED && ISWIFIOK == 0) {
  //if ((ISWIFIOK == 0 )) {
    ip = WiFi.localIP();
    ISWIFIOK = 1;
    ipcheck = cm;
    
    String newIPA;
    //xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]));

    //xprintf(PSTR("HTTP server started\n"));
    newIPA = "IP " + String(ip[2]);
    newIPA += ".";
    newIPA += String(ip[3]);
    if (newIPA != IPA){
      IPA=newIPA;
      sendTelegram(wifi_dns + " SIAP BOS !! IPKU:" +IPA);
     }
  }

#ifdef ESP32
   if (cm - wmc < (uncompress ? 150000 : 30000)) {
#else
   if (cm - wmc < (uncompress ? 500000 : 100000)) {
#endif
     return;
   }

  server.handleClient();
  motionloop();

  #ifdef USEOTA
  if (!uncompress) ArduinoOTA.handle();
  #endif

  wmc = cm;
  if (ISWIFIOK) {
    if (WiFi.status() != WL_CONNECTED) {
      //connectWifi();
      //return;
    }
  }

}


int line_done, ack_waiting = 0;
int ct = 0;
uint32_t gt = 0;
int n = 0;


/*




*/

/*
      =========================================================================================================================================================
*/
int tmax, tmin;

int setupok = 0;

void setupother()
{


  
  if (!SPIFFS.begin()) {
    zprintf(PSTR("Formatting SPIFFS\n"));
    SPIFFS.format();
    SPIFFS.begin();
  }
  /*
    if (!LittleFS.exists("/config.ini")){
      // lets make one
      File f=LittleFS.open("/config.ini","wb");
      for (int i=0;i<sizeof(web_configini);i++)
          f.write(web_configini[i]);
      f.close();
    }
  */

  // important for KARYACNC to know the scale
  //zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));
  readconfigs();
  init_gcode();


  setupwifi(0);





  setupok = 1;
  zprintf(PSTR("start\nok\n"));
}
uint32_t t1;
void setup()
{
  t1 = millis();
  setupother();
  timer_init();
  server.begin();
}

bool pumpon = true;
int llaserOn = 0;
long tm1 = 0;

void loop()
{
  extern bool stopping;
  extern void stopmachine2();
  if (stopping) stopmachine2();
  motionloop();
  uncompress_loop();
  motionloop();
  IR_loop(0);
  servo_loop();
  wifi_loop();
 
}
