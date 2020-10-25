//#define timing
//#define timingG
//#define echoserial

#include "platform.h"
#include "config_pins.h"
#include "common.h"
#include "gcode.h"
#include "temp.h"
#include "timer.h"
#include "eprom.h"
#include "motion.h"
#include "gcodesave.h"
#include "ir_remote.h"
#include "ir_oled.h"


#ifdef USEOTA
#include <ArduinoOTA.h>
#endif
#include<stdint.h>


#ifdef WIFISERVER
//#include <WiFiClient.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
ESP8266WebServer server ( 80 );
#define WIFI_AUTH_OPEN ENC_TYPE_NONE
// ===============================
#elif ESP32
#include <WiFi.h>
#include <WiFiAP.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "SPIFFS.h"
WebServer server ( 80 );
#endif
// ==============================

#include <WiFiClient.h>

#include <FS.h>   // Include the SPIFFS library

#ifdef WEBSOCKSERVER
#include <WebSocketsServer.h>
#endif



uint8_t wfhead = 0;
uint8_t wftail = 0;
uint8_t wfbuf[BUFSIZE];
char wfb[300];
int wfl = 0;
int bpwf = 0;

#ifdef TCPSERVER
WiFiClient client;
WiFiServer servertcp(82);
#endif

#ifdef WEBSOCKSERVER
WebSocketsServer webSocket = WebSocketsServer(81);    // create a websocket server on port 81
#endif

IPAddress ip ;

#define IOT_IP_ADDRESS "172.245.97.171"
long lm = 0;
int ISWIFIOK = 0;
void touchserver(int v, String info) {
#ifndef TOUCHSERVER
  return;
#endif
  if (uncompress)return;
  if (WiFi.status() != WL_CONNECTED)return;
  // put your main code here, to run repeatedly:
  long m = millis();
  if (v || (m - lm > 5000)) {
    char lc, c;
    int cp = 0;
    char cc[20];
    HTTPClient http;
    zprintf(PSTR("Touch server:\n"));
    String url = "http://172.245.97.171/connect?info=" + info + "&ipaddress=" + String((ip[0])) + "." + String( (ip[1])) + "." + String( (ip[2])) + "." + String( (ip[3]));
    http.begin(url);
    int httpCode = http.GET();
    String cmd, par, par2;
    cmd = http.getString();
    //pn("Command:"+cmd);
    if (cmd.indexOf(" ") > 0) {
      par = cmd.substring(cmd.indexOf(" ") + 1);
      cmd = cmd.substring(0, cmd.indexOf(" "));
    }
    http.end();
    if (cmd == "print") {
      par2 = par.substring(par.indexOf(",") + 1);
      par = par.substring(0, par.indexOf(","));
      reload_eeprom();
      zprintf(PSTR("Print:%d Temp:%d\n"), fi(par.toInt()), fi(par2.toInt()));
      if (wifi_gcode == par.toInt()) {
        // no need to redownload
      } else {
        zprintf(PSTR("Downloading..\n"));
        String durl = "http://172.245.97.171/download?act=Download&gid=" + par;
        File f = SPIFFS.open("/gcode.gcode", "w");
        if (f) {
          overridetemp = par2.toInt();
          http.begin(durl);
          int httpCode = http.GET();
          if (httpCode > 0) {
            if (httpCode == HTTP_CODE_OK) {
              //Serial.println(durl);
              //Serial.println(http.getSize());
              http.writeToStream(&f);
            }
          } else {
            zprintf(PSTR("Error download"));//[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
          }
          http.end();
          f.close();
          eepromwrite(EE_gcode, par.toInt());
        }

      }
      extern int8_t RUNNING;
      RUNNING = 1;
      beginuncompress("/gcode.gcode");
    }
    //p("\n");
    lm = m;
  }
}

File fsUploadFile;
//#define NOINTS noInterrupts();
//#define INTS interrupts();
#define NOINTS timerPause();
#define INTS timerResume();



String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}
bool handleFileRead(String path) { // send the right file to the client (if it exists)
  motionloop();
  //NOINTS
  if (SPIFFS.exists(path + ".html")) path += ".html";
  else if (path.endsWith("/config")) path = "/karyaconfig.html";          // If a folder is requested, send the index file
  else if (path.endsWith("/cnc")) path = "/index.html";          // If a folder is requested, send the index file
  else if (path.endsWith("/")) {
    // if wifi not connected redirect to configuration
    if (ISWIFIOK) {
      path = INDEX;          // If a folder is requested, send the index file
      if (!SPIFFS.exists(path))path = "/3d.html";          // If a folder is requested, send the index file
      if (!SPIFFS.exists(path))path = "/index.html";
    }
    else path = "/karyaconfig.html";          // If a folder is requested, send the index file
  }

  xprintf(PSTR("handleFileRead: %s\n"), path.c_str());
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  bool gz = SPIFFS.exists(pathWithGz);
  if (gz || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    File file = SPIFFS.open(gz ? pathWithGz : path, "r");                // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    xprintf(PSTR("\tSent file: %s\n") , path.c_str());
  } else
    xprintf(PSTR("\tFile Not Found: %s\n") , path.c_str());   // If the file doesn't exist, return false

  //INTS
  return false;
}

void handleFileUpload() { // upload a new file to the SPIFFS
  NOINTS
  if (uncompress) {
    server.send(500, "text/plain", "Still PRINTING");
  } else {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (filename == "")filename = "/gcode.gcode";
      if (!filename.startsWith("/")) filename = "/" + filename;
      xprintf(PSTR("handleFileUpload Name: %s\n"), filename.c_str());
      fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
      filename = String();
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // do temploop to avoid overheating
      temp_loop(micros());
      if (fsUploadFile)
        fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
    } else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) {                                   // If the file was successfully created
        fsUploadFile.close();                               // Close the file again
        xprintf(PSTR("handleFileUpload Size: %d\n"), upload.totalSize);
        server.sendHeader("Location", "/upload");     // Redirect the client to the success page
        server.send(303);
      } else {
        server.send(500, "text/plain", "500: couldn't create file");
      }
    }
  }
  INTS
}

#ifdef WEBSOCKSERVER

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      xprintf(PSTR("[%d] Disconnected!\n"), fi(num));
      break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        xprintf(PSTR("[%d] Connected from %d.%d.%d.%d url: %s\n"), fi(num), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]), payload);
        // on connect, send the scale
        zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));
      }
      break;
    case WStype_TEXT:                     // if new text data is received
      //xprintf(PSTR("%s"),payload);
      //webSocket.sendTXT(num, payload);
      for (int i = 0; i < lenght; i++) {
        buf_push(wf, payload[i]);
      }
      //webSocket.broadcastTXT(payload);
      break;
  }
}

#endif
void wifiwr(uint8_t s) {
  wfb[wfl] = s;
  wfl++;
  if (s == '\n') {
    wfb[wfl] = 0;
#ifdef TCPSERVER
    if (client && client.connected()) {
      client.write(wfb, wfl);
    }
#endif
#ifdef WEBSOCKSERVER
    webSocket.broadcastTXT(wfb);
#endif
    wfl = 0;
  }
}

void wifi_push(char c) {
  buf_push(wf, c);
}

void setupwifi(int num) {
  NOINTS


  while (1) {
    xprintf(PSTR("Wifi Initialization\n"));
    if (num) {
      xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );
      break;
      WiFi.disconnect();
      server.close();
    }
    char c = wifi_ap[0];
    if ((c == 0) || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')) {
    } else {
      String("karyacontroller").toCharArray(wifi_dns, 29);
      String("Tenda_AD26C0").toCharArray(wifi_ap, 49);
      String("45712319").toCharArray(wifi_pwd, 19);
      eepromwritestring(470, wifi_dns);
      eepromwritestring(400, wifi_ap);
      eepromwritestring(450, wifi_pwd);
      c = 'a';
    }
    if (c != 0) {
      xprintf(PSTR("Try connect wifi AP:%s \n"), wifi_ap);
      WiFi.mode(WIFI_STA);
      WiFi.begin ( wifi_ap, wifi_pwd);
      int cntr = 30;
      #ifdef IR_OLED_MENU
      d_clear();
      d_text(0, 0, "connecting ...");
      d_text(0, 1, wifi_ap);
      d_show();
      #endif
      while ( WiFi.status() != WL_CONNECTED ) {
        delay ( 500 );
        cntr--;
        if (!cntr)break;
        //xprintf(PSTR("."));
        Serial.print(".");
      }
    }
    String ipa="";
    if (WiFi.status() == WL_CONNECTED) {
      ip = WiFi.localIP();
      ISWIFIOK = 1;
      xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );

      xprintf(PSTR("HTTP server started\n"));
      ipa = "Wifi : " + String(ip[2]);
      ipa += ".";
      ipa += String(ip[3]);

    } else {
      xprintf(PSTR("Access Point Mode\n"));
      WiFi.mode(WIFI_AP);
      ISWIFIOK = 0;
      const char *ssid = "yourAP";
      const char *password = "123456789";
      WiFi.softAP(ssid, password);
      ip = WiFi.softAPIP();
      xprintf(PSTR("AP:%s Ip:%d.%d.%d.%d\n"), ssid, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );
      ipa = String(ssid) + " : " + String(ip[2]);
      ipa += ".";
      ipa += String(ip[3]);
}
#ifdef IR_OLED_MENU
  extern String IPA;  
  IPA=ipa;
    Serial.end();
    hasSerial = 0;
#endif

    server.on("/setconfig", HTTP_GET, []() {                 // if the client requests the upload page
      NOINTS
      server.arg("name").toCharArray(wifi_dns, 29);
      eepromwritestring(470, wifi_dns);
      server.arg("ap").toCharArray(wifi_ap, 49);
      eepromwritestring(400, wifi_ap);
      server.arg("pw").toCharArray(wifi_pwd, 19);
      eepromwritestring(450, wifi_pwd);
      server.send ( 200, "text/html", "OK");
      reload_eeprom();
      INTS
    });
    server.on("/gcode", HTTP_GET, []() {                 // if the client requests the upload page
      if (!uncompress) {
        char gc[100];
        server.arg("t").toCharArray(gc, 99);
        for (int i = 0; i < strlen(gc); i++) {
          buf_push(wf, gc[i]);
        }
        buf_push(wf, '\n');
        server.send ( 200, "text/html", "OK");
      } else {
        server.send(200, "text/plain", "ERR");
      }
    });
    server.on("/speed", HTTP_GET, []() {                 // if the client requests the upload page
      extern uint8_t head, tail, tailok;
      char gc[10];
      server.arg("t").toCharArray(gc, 9);
      extern float f_multiplier;
      f_multiplier = atoi(gc) * 0.01;
      tailok = tail + 5; // need to replanning all moves
      server.send ( 200, "text/html", "OK");

    });

    server.on("/getconfig", HTTP_GET, []() {                 // if the client requests the upload page
      String st = "disconnected";
      if (WiFi.status() == WL_CONNECTED)st = "connected";
      server.send ( 200, "text/html", "['" + String(wifi_ap) + "','" + String(wifi_pwd) + "','" + String(wifi_dns) + "','" + st + "']");
    });
    server.on("/scanwifi", HTTP_GET, []() {
      int n = WiFi.scanNetworks();
      String res = "[";
      if (n == 0) {
      } else {
        for (int i = 0; i < n; ++i) {
          // Print SSID and RSSI for each network found
          if (i)res += ",";
          res += "['" + WiFi.SSID(i) + "','" + WiFi.RSSI(i) + "',";
          if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) res += "''"; else res += "'*'";
          res += "]";
        }
      }
      res += "]";
      server.send(200, "text/html", res);
    });



    server.on("/pauseprint", HTTP_GET, []() {                 // if the client requests the upload page
      //NOINTS
      if (uncompress) {
        ispause = ispause == 0 ? 1 : 0;
        extern int8_t PAUSE;
        PAUSE = ispause;
        server.send ( 200, "text/html", ispause == 1 ? "PAUSED" : "RESUMED");
      } else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/probe", HTTP_GET, []() {
      //NOINTS
      if (!uncompress) {
        docheckendstop(1);
        extern int16_t endstopstatus;
        server.send ( 200, "text/html", endstopstatus < 0 ? "HIGH" : "LOW");
      } else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/resumeprint", HTTP_GET, []() {                 // if the client requests the upload page
      //NOINTS
      if (uncompress) {
        ispause = 0;
        extern int8_t PAUSE;
        PAUSE = ispause;
        server.send ( 200, "text/html", "OK");
      } else {
        server.send(200, "text/plain", "ERR");
      }
      //INTS
    });
    server.on("/startprint", HTTP_GET, []() {                 // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send ( 200, "text/html", "FAIL, STILL PRINTING");
      } else {
        server.send ( 200, "text/html", "OK");
        beginuncompress("/gcode.gcode");
      }
      INTS
    });
    server.on("/startjob", HTTP_GET, []() {                 // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send ( 200, "text/html", "FAIL, STILL PRINTING");
      } else {
        beginuncompress(server.arg("jobname"));
        server.send ( 200, "text/html", "Start Ok");
      }
      INTS
    });
    server.on("/removejob", HTTP_GET, []() {                 // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send ( 200, "text/html", "FAIL, STILL PRINTING");
      } else {
        deletejob(server.arg("jobname"));  
        server.send ( 200, "text/html", "Delete Ok");
      }
      INTS
    });
    server.on("/renamejob", HTTP_GET, []() {                 // if the client requests the upload page
      NOINTS
      if (uncompress) {
        server.send ( 200, "text/html", "FAIL, STILL PRINTING");
      } else {
        SPIFFS.rename(server.arg("jobname"),server.arg("newname"));
        SPIFFS.rename(server.arg("jobname") + ".jpg",server.arg("newname") + ".jpg");
        server.send ( 200, "text/html", "Rename Ok");
      }
      INTS
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
            if (str.length() > 2)str += ",";
            str += "['";
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
      } else {
        server.send(200, "text/plain", "ERR");
      }
      INTS
    });
    server.on("/stopprint", HTTP_GET, []() {                 // if the client requests the upload page
      if (!uncompress) {
        server.send ( 200, "text/html", "FAIL, NOT PRINTING");
      } else {
        server.send ( 200, "text/html", "OK");
        enduncompress();
        extern void stopmachine();
        stopmachine();

      }
    });
    server.on("/home", HTTP_GET, []() {                 // if the client requests the upload page
      addmove(100, 0, 0, 10, 0, 1, 1);
      addmove(100, 0, 0, 10, 0, 1, 0);
      addmove(100, 0, 0, 0, 0, 1, 0);
      server.send ( 200, "text/html", "OK");
      //homing();
    });
    server.on("/jogmove", HTTP_GET, []() {                 // if the client requests the upload page
      server.send ( 200, "text/html", "OK");
      if (!uncompress) {
        float x, y, z;
        x = server.arg("x").toFloat();
        y = server.arg("y").toFloat();
        z = server.arg("z").toFloat();
        addmove(100, x, y, z, 0, 1, 1);
      }
    });
    server.on("/heating", HTTP_GET, []() {                 // if the client requests the upload page
      int temp = 0;
      if (server.arg("t0") == "")temp = 180; else temp = server.arg("t0").toInt();
      server.send ( 200, "text/html", String(Input));
      set_temp(180);
    });
    server.on("/cooling", HTTP_GET, []() {                 // if the client requests the upload page
      server.send ( 200, "text/html", "Cooling");
      set_temp(0);
    });
    server.on("/upload", HTTP_GET, []() {                 // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      server.send ( 200, "text/html", "<form method=\"post\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"name\"> <input class=\"button\" type=\"submit\" value=\"Upload\"></form>" );
    });
    server.on("/delete", HTTP_GET, []() {                 // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      SPIFFS.remove(server.arg("fn"));
      server.send ( 200, "text/html", "Delete " + server.arg("fn"));
    });
    server.on("/rename", HTTP_GET, []() {                 // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      SPIFFS.rename(server.arg("fn"),server.arg("fnew"));
      server.send ( 200, "text/html", "Rename to " + server.arg("fnew"));
    });

    server.on("/upload", HTTP_POST,                       // if the client posts to the upload page
    []() {
      server.sendHeader("Access-Control-Expose-Headers", "Access-Control-*");
      server.sendHeader("Access-Control-Allow-Headers", "Access-Control-*, Origin, X-Requested-With, Content-Type, Accept");
      server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS, HEAD");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200);
    },                          // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload                                    // Receive and save the file
             );


    server.onNotFound([]() {                              // If the client requests any URI
      if (!handleFileRead(server.uri()))                  // send it if it exists
        server.send(404, "text/plain", "404: Not Found ?"); // otherwise, respond with a 404 (Not Found) error
    });
    server.begin();

#ifdef TCPSERVER
    servertcp.begin();
#endif
#ifdef WEBSOCKSERVER
    webSocket.begin();                          // start the websocket server
    webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
#endif


    //if (!num)
#ifdef ESP32
    SPIFFS.begin(true);
#else
    SPIFFS.begin();
#endif
    loadmeshleveling();

    touchserver(1, String(wifi_dns));
#ifdef DISABLESERIAL
    //Serial.end();
#endif

#ifdef USEOTA
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
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
void wifi_loop() {
  uint32_t uc = millis();
  if (uc - wmc < 50) {
    return;
  }
  wmc = uc;
  server.handleClient();
#ifdef WEBSOCKSERVER
  webSocket.loop();
#endif //webserver
#ifdef TOUCHSERVER
  if (sendwait == 1)touchserver(0, String(wifi_dns));
#endif // touch

#ifdef TCPSERVER
  // wait for a new client:

  if (servertcp.hasClient()) {
    if (client)client.stop();
    client = servertcp.available();
    // when the client sends the first byte, say hello:
  }
  if (client) {
    if (client.connected()) {
      while (client.available()) {
        // read the bytes incoming from the client:
        char thisChar = client.read();
        //serialwr(thisChar);
        //xprintf(PSTR("%s"),payload);
        buf_push(wf, thisChar);
        // echo the bytes back to the client:

      }
    } else client.stop();
  }
#endif // tcp server
#ifdef USEOTA
  ArduinoOTA.handle();
#endif
}
#else
void wifi_loop() {}
void setupwifi(int num) {}
#endif

int line_done, ack_waiting = 0;
int ct = 0;
uint32_t gt = 0;
int n = 0;
uint32_t kctr = 0;
int akey = 0, lkey = 0;
int kdl = 200;



/*




*/

/*
      =========================================================================================================================================================
*/
int tmax, tmin;
uint32_t dbtm, dbcm = 0;
char gcode_loop() {

#ifndef ISPC

  /*
    if (micros() - dbtm > 1000000) {
    dbtm = micros();
    extern int32_t dlp;
    extern int subp;
    float f = (cmctr - dbcm);
    f /= XSTEPPERMM;

    //if (f>0)
    //zprintf(PSTR("Subp:%d STEP:%d %f %d\n"),fi(subp),cmctr,ff(f),fi(dlp/8));
    dbcm = cmctr;
    }
  */

  /*
      =========================================================================================================================================================

       KONTROLBOX

      =========================================================================================================================================================
  */

  /*
      =========================================================================================================================================================

       MOTIONLOOP

      =========================================================================================================================================================
  */

  motionloop();
  servo_loop();

  char c = 0;
  {
    if (ack_waiting) {
      zprintf(PSTR("ok\n"));
      ack_waiting = 0;
      n = 1;
    }
    // wifi are first class
#ifdef WIFISERVER
    if (buf_canread(wf)) {
      buf_pop(wf, c);
    } else
#endif
    {
      if (serialav())
      {
        if (n) {
          gt = micros();
          n = 0;
        }
        serialrd(c);
      }
    }
#ifdef USE_SDCARD
    if (sdcardok == 2) {
      // read from the file until there's nothing else in it:
      if (myFile.available()) {
        c = myFile.read();
        //myFile.write(c);
      } else {
        // close the file:
#ifdef POWERFAILURE
        eepromwrite(EE_lastline, fi(0));
#endif
        myFile.close();
        sdcardok = 0;
        zprintf(PSTR("Done\n"));
        c = 0;
      }
    }
#endif

    if (c) {
      if (c == '\n') {
        lineprocess++;
      }

      //#define echoserial
#ifdef echoserial
      serialwr(c);
#endif
      line_done = gcode_parse_char(c);
      if (line_done) {
        ack_waiting = line_done - 1;
#ifdef timingG
        zprintf(PSTR("Gcode:%dus\n"), fi(micros() - gt));
#endif
      }
    }
    //motionloop();
  }
#else
#endif
  return c;
}
int setupok = 0;

void setupother() {
#ifdef USE_SDCARD
  demoSD();
#endif

#ifdef output_enable
  zprintf(PSTR("Init motion\n"));
  initmotion();
  zprintf(PSTR("Init timer\n"));
  timer_init();
  zprintf(PSTR("Init Gcode\n"));
  init_gcode();
  zprintf(PSTR("Init Temp\n"));
  init_temp();
  zprintf(PSTR("Init eeprom\n"));
  reload_eeprom();
#else
  initmotion();
  init_gcode();
  init_temp();
  reload_eeprom();
  timer_init();
#endif
#ifdef spindle_pin
  //delay(1000);
  pinMode(spindle_pin, OUTPUT);
  xdigitalWrite(spindle_pin, LOW);
#endif
  // important for KARYACNC to know the scale
  //zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));



#ifdef WIFISERVER
  setupwifi(0);
#endif



  servo_init();

  setupok = 1;
  zprintf(PSTR("start\nok\n"));

}
uint32_t t1;
void setup() {
  // put your setup code here, to run once:
  //  Serial.setDebugOutput(true);
  IR_setup();
  setup_oled();
  
  #ifndef IR_OLED_MENU  
  serialinit(BAUDRATE); //115200);
  #endif
  t1 = millis();
  //while (!Serial.available())continue;

  setupother();


}


void loop() {
  //demo();

  if (setupok) {
#ifdef FASTBUFFERFILL
    for (int ff = FASTBUFFERFILL + 1; ff; ff--)
#endif
      char c = gcode_loop();
    IR_loop();
#ifdef WIFISERVER
    wifi_loop();
    if (c == 0)uncompress_loop();
#endif
  }
}
