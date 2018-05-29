
//#define timing
//#define timingG
//#define echoserial


#include "config_pins.h"
#include "common.h"
#include "gcode.h"
#include "temp.h"
#include "timer.h"
#include "eprom.h"
#include "motion.h"

#include<stdint.h>


#ifdef WIFISERVER
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>   // Include the SPIFFS library
#include <WebSocketsServer.h>

#include <TelegramBot.h>

uint8_t wfhead = 0;
uint8_t wftail = 0;
uint8_t wfbuf[BUFSIZE];
char wfb[300];
int wfl = 0;
int bpwf = 0;
ESP8266WebServer server ( 80 );
WebSocketsServer webSocket = WebSocketsServer(81);    // create a websocket server on port 81

File fsUploadFile;

String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}
bool handleFileRead(String path) { // send the right file to the client (if it exists)
  xprintf(PSTR("handleFileRead: %s\n"), path.c_str());
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    xprintf(PSTR("\tSent file: %s\n") , path.c_str());
    return true;
  }
  xprintf(PSTR("\tFile Not Found: %s\n") , path.c_str());   // If the file doesn't exist, return false
  return false;
}

void handleFileUpload() { // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    xprintf(PSTR("handleFileUpload Name: %s\n"), filename.c_str());
    fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
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
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) { // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      xprintf(PSTR("[%d] Disconnected!\n"), fi(num));
      break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        xprintf(PSTR("[%d] Connected from %d.%d.%d.%d url: %s\n"), fi(num), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]), payload);
      }
      break;
    case WStype_TEXT:                     // if new text data is received
      xprintf(PSTR("[%d] get Text: %s\n"), fi(num), payload);
      //webSocket.sendTXT(num, payload);
      for (int i = 0; i < lenght; i++) {
        buf_push(wf, payload[i]);
      }
      //webSocket.broadcastTXT(payload);
      break;
  }
}

void wifiwr(uint8_t s) {
  wfb[wfl] = s;
  wfl++;
  if (s == '\n') {
    wfb[wfl] = 0;
    wfl = 0;
    webSocket.broadcastTXT(wfb);
  }
}

String token = "540208354:AAEEbjIZGymE5Hfifcn9lVfVfCEkUQ2BCeg"   ; // REPLACE myToken WITH YOUR TELEGRAM BOT TOKEN
const char BotToken[] = "540208354:AAEEbjIZGymE5Hfifcn9lVfVfCEkUQ2BCeg";

#include <WiFiClientSecure.h>


WiFiClientSecure net_ssl;
TelegramBot bot (BotToken, net_ssl);

void setupwifi() {

  xprintf(PSTR("Try connect wifi AP:%s \n"), wifi_ap);
  WiFi.mode(WIFI_STA);
  WiFi.begin ( wifi_ap, wifi_pwd);
  int cntr = 40;
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    cntr--;
    if (!cntr)break;
    xprintf(PSTR("."));
  }

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );

    if ( MDNS.begin ( wifi_dns) ) {
      xprintf(PSTR("MDNS responder started %s\n"), wifi_dns);
    }

    server.on("/upload", HTTP_GET, []() {                 // if the client requests the upload page
      if (!handleFileRead("/upload.html"))                // send it if it exists
        server.send ( 200, "text/html", "<form method=\"post\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"name\"> <input class=\"button\" type=\"submit\" value=\"Upload\"></form>" );
    });

    server.on("/upload", HTTP_POST,                       // if the client posts to the upload page
    []() {
      server.send(200);
    },                          // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload                                    // Receive and save the file
             );


    server.onNotFound([]() {                              // If the client requests any URI
      if (!handleFileRead(server.uri()))                  // send it if it exists
        server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
    });
    server.begin();

    webSocket.begin();                          // start the websocket server
    webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
    MDNS.addService("http", "tcp", 80);

#ifdef TELEGRAM
    bot.begin();
    char buf[46];
    sprintf(buf, "CNC:%s http://%d.%d.%d.%d", wifi_dns, ip[0], ip[1], ip[2], ip[3] );

    if (strlen(wifi_telebot))bot.sendMessage(wifi_telebot, buf);
#endif
    xprintf(PSTR("HTTP server started\n"));

  }

  SPIFFS.begin();
}

void wifi_loop() {

  webSocket.loop();
  server.handleClient();
  // if there is an incoming message...
  // its blocking/long time so no use
  /* message m = bot.getUpdates(); // Read new messages
    if ( m.chat_id != 0 ) { // Checks if there are some updates
     Serial.println(m.text);
     bot.sendMessage(m.chat_id, m.text);  // Reply to the same chat with the same text
    }
  */
}
#else
#define wifi_loop()
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
#ifdef LCDDISPLAY
#include  <Wire.h>
#include  <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(LCDDISPLAY, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

void menu_up() {

}
void menu_down() {

}
void menu_click() {

}
void menu_back() {

}

#define KBOX_KEY_ACT(k)   case k: KBOX_KEY##k##_ACTION;break;
#define KBOX_KEY1_ACTION menu_up()
#define KBOX_KEY2_ACTION menu_down()
#define KBOX_KEY3_ACTION menu_click()
#define KBOX_KEY4_ACTION menu_back()

#define KBOX_DO_ACT  KBOX_KEY_ACT(1) KBOX_KEY_ACT(2) KBOX_KEY_ACT(3) KBOX_KEY_ACT(4)


void oledwr(uint8_t c) {
  lcd.write(c);
}
#define oprintf(...)   sendf_P(oledwr, __VA_ARGS__)
#define gotoxy(x,y) lcd.setCursor(x,y)
#define oclear() lcd.clear()


//------------------------------------------------------------------------------
void setupdisplay() {

  lcd.begin();                      // initialize the lcd
  lcd.backlight();
  gotoxy(0, 0);
  oprintf(PSTR("Karyacontroller"));
}
uint32_t sw, next_lcd = 0;
void display_loop() {

  uint32_t lcm = millis();
  if (lcm - next_lcd < 500) return;
  next_lcd = lcm; // each half second
  if (sw++ > 3)sw = 0;
  switch (sw) {
    case 0:
      gotoxy(0, 0);
#ifdef USE_SDCARD
      if (sdcardok == 1) {
        oprintf(PSTR("SD:%d lines"), fi(linecount));
      } else if (sdcardok == 2) {
        oprintf(PSTR("Printing:%d"), fi(lineprocess));
      } else
#endif
      {
        oprintf(PSTR("Suhu:%f         "), ff(Input));
      }
      break;
    case 1:
      gotoxy(0, 1);
      //              ----------------
#ifdef USE_SDCARD
      if (sdcardok == 1) {
        oprintf(PSTR("Home Heat Prn -"));
      }
      else if (sdcardok == 2) {
        oprintf(PSTR("- -  Pause Off"));
      }
#endif
      {
        oprintf(PSTR("Home Heat SD Off"));
      }
      break;
  }
}
void control_loop() {
  /*
    #ifdef OLED_CONTROL_PIN

    if (millis() - kctr > kdl) {
    kctr = millis();
    #ifdef ISRTEMP
    int key = vanalog[OLED_CONTROL_PIN];
    ADCREAD(OLED_CONTROL_PIN)
    #else
    int key = analogRead(OLED_CONTROL_PIN) >> ANALOGSHIFT;
    #endif

    switch (key) {
        KBOX_DO_CHECK  // standart 4 key control box

      case 1021 ... 1023:
        if (lkey) {
          zprintf(PSTR("KKey:%d\n"), fi(lkey));
          switch (lkey) {
            KBOX_DO_ACT
          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
    zprintf(PSTR("Key:%d\n"), fi(key));
    }
    #endif
  */
}
#else
#define setupdisplay()
#define display_loop()
#define control_loop()

#endif
/*
      =========================================================================================================================================================
*/
int tmax, tmin;
uint32_t dbtm, dbcm = 0;
void gcode_loop() {

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
#ifdef KBOX_PIN

  if (millis() - kctr > kdl) {
    kctr = millis();
#ifdef ISRTEMP
    int key = vanalog[KBOX_PIN];
    ADCREAD(KBOX_PIN)
#else
    int key = analogRead(KBOX_PIN) >> ANALOGSHIFT;
#endif

    switch (key) {
        KBOX_DO_CHECK
      case 1021 ... 1023:
        if (lkey) {
          //zprintf(PSTR("LKey:%d\n"), fi(lkey));
          switch (lkey) {
            case 4: zprintf(PSTR("HOMING\n")); homing(); break;
            case 3: zprintf(PSTR("HEATING\n")); set_temp(190); break;
            case 2: if (sdcardok) {
                sdcardok = sdcardok == 1 ? 2 : 1;
                zprintf(PSTR("SD\n"));
              } else demoSD(); break;
            case 1: RUNNING = 0; sdcardok = 0; zprintf(PSTR("STOP\n")); power_off(); break;

          }
        }
        lkey = 0;
        kdl = 200;
        break;
    }
#ifdef KBOX_SHOW_VALUE
    zprintf(PSTR("Key:%d\n"), fi(key));
#endif

  }
#endif
  /*
      =========================================================================================================================================================
  */

  /*
      =========================================================================================================================================================

       MOTIONLOOP

      =========================================================================================================================================================
  */
  int32_t zz;
#ifdef timing
  uint32_t t1 = micros();
  if (motionloop())
  {
    uint32_t t2 = micros() - t1;
    tmax = fmax(t2, tmax);
    tmin = fmin(t2, tmin);

    if (ct++ > 1) {
      zprintf(PSTR("%d %dus\n"), fi(tmin), fi(tmax));
      tmax = 0;
      tmin = 1000;
      ct = 0;
    }
  }
#else
  for (int i = 0; i < 5; i++) {
    if (!motionloop())break;
  }
#endif
  servo_loop();
  if (ack_waiting) {
    zprintf(PSTR("ok\n"));
    ack_waiting = 0;
    n = 1;
  }


  char c = 0;
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
#else
#endif

}
int setupok = 0;

void setupother() {
#ifdef USE_SDCARD
  demoSD();
#endif
#ifdef output_enable
  zprintf(PSTR("Init motion\n"));
  initmotion();
  zprintf(PSTR("Init Gcode\n"));
  init_gcode();
  zprintf(PSTR("Init Temp\n"));
  init_temp();
  zprintf(PSTR("Init eeprom\n"));
  reload_eeprom();
  //zprintf(PSTR("Init timer\n"));
  //timer_init();
#else
  initmotion();
  init_gcode();
  init_temp();
  reload_eeprom();
  timer_init();
#endif
#ifdef WIFISERVER
  setupwifi();
#endif


  setupdisplay();

  servo_init();
#ifdef KBOX_PIN
#ifdef __ARM__
  pinMode(KBOX_PIN, INPUT_ANALOG);
#else
  pinMode(KBOX_PIN, INPUT_PULLUP);
#endif
#ifdef ISRTEMP
  vanalog[KBOX_PIN] = 1023; // first read is error, throw it
#endif
#endif


  setupok = 0;
  zprintf(PSTR("start\nok\n"));

}
uint32_t t1;
void setup() {
  // put your setup code here, to run once:
  //  Serial.setDebugOutput(true);
  serialinit(115200);//115200);
  t1 = millis();
  //while (!Serial.available())continue;
#ifndef DELAYEDSETUP
  setupother();
#endif
}
void loop() {
  //demo();
#ifdef DELAYSETUP
  if (!setupok && (t1 - millis() > DELAYSETUP * 1000))setupother();
#endif
  if (setupok) {
    gcode_loop();
    control_loop();
    display_loop();
    wifi_loop();
  }
}
