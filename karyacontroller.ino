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

//#define USE_CLIENTSSL true  
#include "common.h"
#include "gcode.h"
#include "temp.h"
#include "timer.h"
#include "eprom.h"
#include "gcodesave.h"
#include "ir_remote.h"
#include "ir_oled.h"
#include <stdint.h>


#ifndef WIFISERVER
// provide empty implementation
inline void wifi_loop() {}
inline void setupwifi(int num) {}
#else
#ifdef USEOTA
#include <ArduinoOTA.h>
#endif
#include <FS.h> // Include the SPIFFS library

#ifdef ESP8266

#define USE_CLIENTSSL true  



#include <ESP8266WiFi.h>
//#include <WiFiClientSecure.h>
//#include <TelegramBot.h>
//WiFiClientSecure net_ssl;
//TelegramBot myBot (0, net_ssl);

#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
//#include <ESP8266mDNS.h>

ESP8266WebServer server(80);
#define WIFI_AUTH_OPEN ENC_TYPE_NONE
// ===============================
#elif ESP32
#include <WiFi.h>
#include <WiFiAP.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "SPIFFS.h"
WebServer server(80);
#endif
// ==============================

#ifdef TCPSERVER
#include <WiFiClient.h>
#endif
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
WebSocketsServer webSocket = WebSocketsServer(81); // create a websocket server on port 81
#endif
String bot_token;
String report_id;

/*
/* This is used with ESP8266 platform only * /
static const char telegram_cert[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIEADCCAuigAwIBAgIBADANBgkqhkiG9w0BAQUFADBjMQswCQYDVQQGEwJVUzEh
MB8GA1UEChMYVGhlIEdvIERhZGR5IEdyb3VwLCBJbmMuMTEwLwYDVQQLEyhHbyBE
YWRkeSBDbGFzcyAyIENlcnRpZmljYXRpb24gQXV0aG9yaXR5MB4XDTA0MDYyOTE3
MDYyMFoXDTM0MDYyOTE3MDYyMFowYzELMAkGA1UEBhMCVVMxITAfBgNVBAoTGFRo
ZSBHbyBEYWRkeSBHcm91cCwgSW5jLjExMC8GA1UECxMoR28gRGFkZHkgQ2xhc3Mg
MiBDZXJ0aWZpY2F0aW9uIEF1dGhvcml0eTCCASAwDQYJKoZIhvcNAQEBBQADggEN
ADCCAQgCggEBAN6d1+pXGEmhW+vXX0iG6r7d/+TvZxz0ZWizV3GgXne77ZtJ6XCA
PVYYYwhv2vLM0D9/AlQiVBDYsoHUwHU9S3/Hd8M+eKsaA7Ugay9qK7HFiH7Eux6w
wdhFJ2+qN1j3hybX2C32qRe3H3I2TqYXP2WYktsqbl2i/ojgC95/5Y0V4evLOtXi
EqITLdiOr18SPaAIBQi2XKVlOARFmR6jYGB0xUGlcmIbYsUfb18aQr4CUWWoriMY
avx4A6lNf4DD+qta/KFApMoZFv6yyO9ecw3ud72a9nmYvLEHZ6IVDd2gWMZEewo+
YihfukEHU1jPEX44dMX4/7VpkI+EdOqXG68CAQOjgcAwgb0wHQYDVR0OBBYEFNLE
sNKR1EwRcbNhyz2h/t2oatTjMIGNBgNVHSMEgYUwgYKAFNLEsNKR1EwRcbNhyz2h
/t2oatTjoWekZTBjMQswCQYDVQQGEwJVUzEhMB8GA1UEChMYVGhlIEdvIERhZGR5
IEdyb3VwLCBJbmMuMTEwLwYDVQQLEyhHbyBEYWRkeSBDbGFzcyAyIENlcnRpZmlj
YXRpb24gQXV0aG9yaXR5ggEAMAwGA1UdEwQFMAMBAf8wDQYJKoZIhvcNAQEFBQAD
ggEBADJL87LKPpH8EsahB4yOd6AzBhRckB4Y9wimPQoZ+YeAEW5p5JYXMP80kWNy
OO7MHAGjHZQopDH2esRU1/blMVgDoszOYtuURXO1v0XJJLXVggKtI3lpjbi2Tc7P
TMozI+gciKqdi0FuFskg5YmezTvacPd+mSYgFFQlq25zheabIZ0KbIIOqPjCDPoQ
HmyW74cNxA9hi63ugyuV+I6ShHI56yDqg+2DzZduCLzrTia2cyvk0/ZM/iZx4mER
dEr/VxqHD3VILs9RaRegAhJhldXRQLIQTO7ErBBDpqWeCtWVYpoNz4iCxTIM5Cuf
ReYNnyicsbkqWletNw+vHX/bvZ8=
-----END CERTIFICATE-----
)EOF";


#include <time.h>
BearSSL::Session   session;
BearSSL::X509List  certificate(telegram_cert);

void sendTelegram(String stext){

 // Sync time with NTP, to check properly Telegram certificate
    configTime("CET-1CEST,M3.5.0,M10.5.0/3", "time.google.com", "time.windows.com", "pool.ntp.org");
  //Set certficate, session and some other base client properies
    BearSSL::WiFiClientSecure client;
    client.setSession(&session);
    client.setTrustAnchors(&certificate);

    IPAddress server(149,154,167,220);  

    int r=0;
    while (!client.connect(server, 443)){
      delay(100);
      if (r++>20)return;
    }

    String Link="/bot5069763317:AAHa9nRGslJYI2u073P_vebfexIMgTfUJhY/sendMessage";
    String text="{\"chat_id\":\"447996950\",\"text\":\""+stext+"\"}";
    client.print(String("POST ") + Link + " HTTP/1.1\r\n" +
               "Host: api.telegram.org\r\n" +
               "Content-Type: application/json"+ "\r\n" +
               "Content-Length: "+text.length() + "\r\n\r\n" +
               text + "\r\n" +
               "Connection: close\r\n\r\n");
}
*/
#include "UrlEncode.h"
IPAddress serverIP;
String karyacncIP;
int connect_timeout=10;
IPAddress fixedip;

  extern float stepmmy;
  extern bool isRotary;
void sendTelegram(String stext){
    int ll=karyacncIP.length();
    if (ll || serverIP[0]>1){
        HTTPClient http;
        String Link="http://";
        if (ll)Link+=karyacncIP;
        else Link+=String(serverIP[0])+"."+String(serverIP[1])+"."+String(serverIP[2])+"."+String(serverIP[3]);
        Link+=":8888/t?bot="+bot_token+"&id="+report_id+"&t="+urlEncode(stext);
        http.begin(Link);
        int httpCode = http.GET();
        http.end();
    }
}
void downloadFile(String durl,String outf){
    /*
//  String durl = "http://172.245.97.171/download?act=Download&gid=" + par;
    HTTPClient http;

    if (SPIFFS.exists(outf))return; // if exist, just exit
    File f = SPIFFS.open(outf, "w");
    if (f) {
      http.begin(durl);
      int httpCode = http.GET();
      if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK) {
          http.writeToStream(&f);
        }
      }
      http.end();
      f.close();
    }
    */
}

int strToPin(File f1,char u='\n'){
    String s2=f1.readStringUntil(u);
    if(s2=="D1")return D1;    
    if(s2=="D2")return D2;    
    if(s2=="D3")return D3;    
    if(s2=="D4")return D4;    
    if(s2=="D5")return D5;    
    if(s2=="D6")return D6;    
    if(s2=="D7")return D7;    
    if(s2=="D8")return D8;    
    if(s2=="D0")return D0;    
    if(s2=="TX")return TX;    
    if(s2=="RX")return RX;    
    if(s2=="A0")return A0;   
    if(s2=="?")return 255;   
    return -1;
}
int strToMt(String s2){
    if (s2=="X")return 0;
    if (s2=="Y")return 1;
    if (s2=="Z")return 2;
    if (s2=="R")return 3;
    return 0;       
}

bool makezeropoint;
extern int zerocrosspin;
void readConfig(String fn){
  if (!SPIFFS.exists(fn))return;
  File f1;
  f1 = SPIFFS.open(fn, "r");
  if (!f1) {
    //zprintf(PSTR("File Config found\n"));
    return;
  }
  String s1,s2,s3,s4,s5,s6;

  while  (f1.available()) {
    s1=f1.readStringUntil('=');
    if (s1==";"){
        s1=f1.readStringUntil('\n');
    } else if (s1=="connect_timeout"){
        connect_timeout=f1.readStringUntil('\n').toInt();  
    } else if (s1=="testlaser"){
        extern int testlaserdur;
        testlaserdur=3000+f1.readStringUntil('\n').toInt();  
    } else if (s1=="stepdelay"){
        extern int stepdelay;
        stepdelay=f1.readStringUntil('\n').toInt();  
    } else if (s1=="fixed_ip"){
        if (!fixedip.fromString(f1.readStringUntil('\n')))fixedip[0]=0;
    }else if (s1=="SSID"){
        wifi_ap=f1.readStringUntil('\n');wifi_ap.trim();      
    } else if (s1=="PASSWORD"){
        wifi_pwd=f1.readStringUntil('\n');wifi_pwd.trim();     
    } else if (s1=="machine_name"){
        wifi_dns=f1.readStringUntil('\n');wifi_dns.trim();  
    } else if (s1=="motor"){
        // no,step/mm,speed
        s2=f1.readStringUntil(',');
        int mt=strToMt(s2);
        stepmmx[mt]=String(f1.readStringUntil(',')).toFloat(); // stepmmx
        maxf[mt]=String(f1.readStringUntil(',')).toFloat(); // maxf
        maxa[mt]=String(f1.readStringUntil(',')).toFloat(); // maxa
        xback[mt]=String(f1.readStringUntil('\n')).toFloat(); // xback  
        if (maxa[mt]<110)maxa[mt]=110;
        if (maxf[mt]<5)maxa[mt]=5;
    } else if (s1=="motor_pin"){ 
        s2=f1.readStringUntil(',');
        int mt=strToMt(s2);
        mdir_pin[mt]=strToPin(f1,','); 
        mstep_pin[mt]=strToPin(f1); 
    } else if (s1=="home"){        
        ax_home[0]=String(f1.readStringUntil(',')).toFloat(); // maxf
        ax_home[1]=String(f1.readStringUntil(',')).toFloat(); // maxa
        ax_home[2]=String(f1.readStringUntil('\n')).toFloat(); // xback  
    } else if (s1=="home_ofs"){        
        axisofs[0]=String(f1.readStringUntil(',')).toFloat(); // maxf
        axisofs[1]=String(f1.readStringUntil(',')).toFloat(); // maxa
        axisofs[2]=String(f1.readStringUntil('\n')).toFloat(); // xback  
    } else if (s1=="home_feed"){
        homingspeed=f1.readStringUntil('\n').toFloat();      
    } else if (s1=="bot_token"){
        bot_token=f1.readStringUntil('\n');    
    } else if (s1=="master"){
        karyacncIP=f1.readStringUntil('\n');    
    } else if (s1=="report_id"){
        report_id=f1.readStringUntil('\n');    
    } else if (s1=="limit_pin"){
        limit_pin=strToPin(f1);    
    } else if (s1=="zerocross_pin"){
        zerocrosspin=strToPin(f1);  
    } else if (s1=="mode"){
        s2=f1.readStringUntil('\n');
        if (s2=="router")lasermode=0;
        if (s2=="laser")lasermode=1;
        if (s2=="plasma")lasermode=2;
    } else if (s1=="pwm_pow"){
        extern float pwm_pow;
        pwm_pow=f1.readStringUntil('\n').toFloat();
    } else if (s1=="lcd_addr"){
        lcd_rst=f1.readStringUntil('\n').toInt();   
    } else if (s1=="ir_pin"){
        lIR_KEY=strToPin(f1);  
    } else if (s1=="lcd_rst"){
        lcd_rst=strToPin(f1);   
    } else if (s1=="lcd_sda"){
        lcd_sda=strToPin(f1);   
    } else if (s1=="lcd_sda2"){
        lcd_sda2=strToPin(f1);   
    } else if (s1=="lcd_contrast"){
        lcd_contrast=f1.readStringUntil('\n').toFloat();   
    } else if (s1=="lcd_scl"){
        lcd_scl=strToPin(f1);   
    }  else if (s1=="lcd_cs"){
        // dont care about CS for now
        s2=f1.readStringUntil('\n');   
    }  else if (s1=="lcd_type"){
        s2=f1.readStringUntil('\n');  
        if (s2=="NK1661")lcd_kind=KIND_NK1661;else
        if (s2=="NK1202")lcd_kind=KIND_NK1202;else 
        if (s2=="NK6100")lcd_kind=KIND_NK6100; else
        if (s2=="OLED1306")lcd_kind=KIND_OLED1306; else
        if (s2=="LCD2004")lcd_kind=KIND_LCD2004; else
        if (s2=="ST7565")lcd_kind=KIND_ST7565; else
        if (s2=="ST7735")lcd_kind=KIND_ST7735; 
    }  else if (s1=="tool_pin"){
        atool_pin=strToPin(f1);    
    }  else if (s1=="pwm_pin"){
        pwm_pin=strToPin(f1);    
    } else if (s1=="corner"){
        xycorner=f1.readStringUntil('\n').toInt();    
    } else if (s1=="min_pwm_clock"){
        extern int min_pwm_clock;
        // in uS        
        min_pwm_clock=5*f1.readStringUntil('\n').toFloat();    
    } else if (s1=="lscale"){
        Lscale=f1.readStringUntil('\n').toFloat();    
    } else if (s1=="skew_y"){
        skew_y=f1.readStringUntil('\n').toFloat();    
    } else if (s1=="thc_up"){
        thc_up=f1.readStringUntil('\n').toFloat();    
    } else if (s1=="thc_ofs"){
        thc_ofs=f1.readStringUntil('\n').toFloat(); 
    } else if (s1=="makezeropoint"){
        makezeropoint=f1.readStringUntil('\n')=="Y";
    } else if (s1=="laser_on"){
        extern bool TOOLONS[3];
        TOOLONS[1]=f1.readStringUntil('\n')=="H"?HIGH:LOW;
    } else if (s1=="plasma_on"){
        extern bool TOOLONS[3];
        TOOLONS[2]=f1.readStringUntil('\n')=="H"?HIGH:LOW;
    } else if (s1=="trimmer_on"){
        extern bool TOOLONS[3];
        TOOLONS[0]=f1.readStringUntil('\n')=="H"?HIGH:LOW;
    } else if (s1=="showkey"){
        extern bool showremotekey;
        showremotekey=f1.readStringUntil('\n')=="Y";
    } else if (s1=="water"){
        if (f1.readStringUntil('\n')=="Y"){
            water_pin=A0;
            //pinMode(A0,INPUT_PULLDOWN);
        } else water_pin=-1;
    } else if (s1=="thc"){
        thc_enable=f1.readStringUntil('\n')=="Y";
    } else if (s1=="temp_pin"){
        ltemp_pin=strToPin(f1);    
    } else if (s1=="temp_limit"){
        temp_limit=f1.readStringUntil('\n').toInt();    
    } else if (s1=="buzzer"){
        BUZZER_ERR=strToPin(f1);   
        pinmode(BUZZER_ERR,OUTPUT); 
    } else {
        f1.readStringUntil('\n');
    }
  }
  f1.close();
}
void printmotor(File f1,int mt){
  f1.print(stepmmx[mt],3);f1.print(",");
  f1.print(maxf[mt]);f1.print(",");
  f1.print(maxa[mt]);f1.print(",");
  f1.println(xback[mt],3);
}
void saveconfigs(){
  File f1;
  f1 = SPIFFS.open("/custom.ini", "w");
  f1.print("SSID=");f1.println(wifi_ap);
  f1.print("PASSWORD=");f1.println(wifi_pwd);
  f1.print("machine_name=");f1.println(wifi_dns);
  f1.print("motor=X,"); printmotor(f1,0);
  f1.print("motor=Y,"); printmotor(f1,1);
  f1.print("motor=Z,"); printmotor(f1,2);
  f1.print("motor=R,"); printmotor(f1,3);
  f1.print("corner=");f1.println(xycorner);
  f1.print("lscale=");f1.println(Lscale);
  f1.print("skew_y=");f1.println(skew_y);
  f1.print("thc_up=");f1.println(thc_up);
  f1.print("thc_ofs=");f1.println(thc_ofs);
  f1.print("home=");f1.print(ax_home[0],2);f1.print(",");
                    f1.print(ax_home[1],2);f1.print(",");
                    f1.println(ax_home[2],2);
  f1.print("home_ofs=");f1.print(axisofs[0],2);f1.print(",");
                    f1.print(axisofs[1],2);f1.print(",");
                    f1.println(axisofs[2],2);
  f1.close();
  pre_motion_set();
}
void readconfigs(){
  //noInterrupts();
  mstep_pin[2]=-1;
  mdir_pin[2]=-1;
  skew_y=0;
  fixedip[0]=0;
  extern float pwm_pow;
  pwm_pow=1;
  
  karyacncIP="";
  wifi_ap="";
  lcd_sda2=255;
  lcd_scl=255;
  
  pwm_pin=-1;

  TOOLONS[0]=HIGH;
  TOOLONS[1]=HIGH;
  TOOLONS[2]=HIGH;
  
  zerocrosspin=-1;
  lcd_kind=KIND_NK1661;
  lcd_sda=TX;
  lcd_scl=RX;
  lIR_KEY=TX;
  lcd_rst=D1;
  makezeropoint=true;
  
  Lscale=1;
  xycorner=20;
  readConfig("/config.ini");
  readConfig("/custom.ini");
  // calculate something
  TOOLON=TOOLONS[lasermode];
  pre_motion_set();
  restartLCD();  
  init_temp();
  if (pwm_pin==-1)pwm_pin=atool_pin;
  pinmode(atool_pin,OUTPUT);
  pinmode(pwm_pin,OUTPUT);
  xdigitalWrite(atool_pin,!TOOLON);
  xdigitalWrite(pwm_pin,!TOOLON);
  

  stepmmy=stepmmx[1];
  isRotary=false;
  // Set the Telegram bot properies
  //myBot.token=Buf;  
  //myBot.sendMessage(report_id, "test");
  //interrupts();  
}
IPAddress ip;
// Server
#define IOT_IP_ADDRESS "172.245.97.171"
long lm = 0;
int ISWIFIOK = 0;
void touchserver(int v, String info)
{
#ifndef TOUCHSERVER
  return;
#else
  if (uncompress)
    return;
  if (WiFi.status() != WL_CONNECTED)
    return;
  // put your main code here, to run repeatedly:
  long m = millis();
  if (v || (m - lm > 5000)) {
    char lc, c;
    int cp = 0;
    char cc[20];
    HTTPClient http;
    zprintf(PSTR("Touch server:\n"));
    String url = "http://172.245.97.171/connect?info=" + info + "&ipaddress=" + String((ip[0])) + "." + String((ip[1])) + "." + String((ip[2])) + "." + String((ip[3]));
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
      }
      else {
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
          }
          else {
            zprintf(PSTR("Error download")); //[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
          }
          http.end();
          f.close();
          eepromwrite(EE_gcode, par.toInt());
        }
      }
      extern int8_t RUNNING;
      RUNNING = 1;
      beginuncompress("/gcode.gcode",false,0);
    }
    //p("\n");
    lm = m;
  }
#endif
}

File fsUploadFile;
//#define NOINTS noInterrupts();
//#define INTS interrupts();
#define NOINTS timerPause();
#define INTS timerResume();
//#define  NOINTS
//#define INTS

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
  else if (path=="/config")
  {
    path = "/karyaconfig.html"; // If a folder is requested, send the index file
    if (!SPIFFS.exists(path)){
        server.send(200,"text/html",web_config);
        return true;
    }
  }
  else if (path=="/karyacnc.7z" || path=="/karyacnc.zip")
  {
      #ifdef karyacnc_zip_len
        server.sendHeader("Content-Disposition","attachment; filename=karyacnc.7z");
        server.send(200,"application/x-zip",karyacnc_zip,karyacnc_zip_len);
        return true;
      #endif
  }
    else if (path=="/config.ini")
  {
    if (!SPIFFS.exists(path)){
        server.send(200,"text/plain",web_configini);
        return true;
    }
  }
  else if (path=="/--gcodex.js")
  {
    if (!SPIFFS.exists(path)){
        server.send(200,"application/javascript",web_gcodex);
        return true;
    }
  }  else if (path=="/websocket.js")
  {
    if (!SPIFFS.exists(path)){
        server.send(200,"application/javascript",web_websocket);
        return true;
    }
  }
  else if (path.endsWith("/cnc"))
    {
        path = "/cnc.html"; // If a folder is requested, send the index file
         if (!SPIFFS.exists(path)){
            server.send(200,"text/html",web_cnc);
            return true;
        }
    } 
  else if (path.endsWith("/")) {
        path = "/jobs.html"; // If a folder is requested, send the index file
         if (!SPIFFS.exists(path)){
            server.send(200,"text/html",web_index);
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

bool isconfig,isfirmware;
void handleFileUpload()
{ // upload a new file to the SPIFFS
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
      isconfig=filename.endsWith(".ini");
      isfirmware=filename.endsWith(".bin");
      if (isconfig && !filename.startsWith("/custom"))filename="/config.ini";
      if (isfirmware)filename="/firmware.bin";
      xprintf(PSTR("handleFileUpload Name: %s\n"), filename.c_str());
      fsUploadFile = SPIFFS.open(filename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
      filename = String();
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
      // do temploop to avoid overheating
      temp_loop(micros());
      if (fsUploadFile)
        fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
    }
    else if (upload.status == UPLOAD_FILE_END) {
      if (fsUploadFile) { // If the file was successfully created
        fsUploadFile.close(); // Close the file again
        //xprintf(PSTR("handleFileUpload Size: %d\n"), upload.totalSize);
        server.sendHeader("Location", "/upload"); // Redirect the client to the success page
        server.send(303);
        if (isconfig)readconfigs();
      }
      else {
        server.send(500, "text/plain", "500: couldn't create file");
      }
    }
  }
  INTS
}

#ifdef WEBSOCKSERVER

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t lenght)
{ // When a WebSocket message is received
  switch (type) {
    case WStype_DISCONNECTED: // if the websocket is disconnected
      //xprintf(PSTR("[%d] Disconnected!\n"), fi(num));
      break;
    case WStype_CONNECTED: { // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        //xprintf(PSTR("[%d] Connected from %d.%d.%d.%d url: %s\n"), fi(num), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]), payload);
        // on connect, send the scale
        //zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));
      } break;
    case WStype_TEXT: // if new text data is received
      //xprintf(PSTR("%s"),payload);
      //webSocket.sendTXT(num, payload);
      // if runnning job, ignore command from all channel
      if (!uncompress) {
        for (int i = 0; i < lenght; i++) {
          buf_push(wf, payload[i]);
        }
      }
      //webSocket.broadcastTXT(payload);
      break;
  }
}
#endif


void wifiwr(uint8_t s)
{
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
    if (!uncompress)webSocket.broadcastTXT(wfb);
#endif
    wfl = 0;
  }
}

void wifi_push(char c)
{
  buf_push(wf, c);
}

void connectWifi(int ret = 1)
{
  if (ret) {
    WiFi.reconnect();
    return;
  }
  
  //xprintf(PSTR("Try connect wifi AP:%s \n"), wifi_ap);
  WiFi.mode(WIFI_STA);
  if (fixedip[0]>0){
      IPAddress gateway;
      gateway=fixedip;
      gateway[3]=1;
      IPAddress subnet(255, 255, 255, 0);
      WiFi.config(fixedip,gateway,subnet);
  }
  WiFi.begin(wifi_ap, wifi_pwd);

  int cntr = connect_timeout;

  while (WiFi.status() != WL_CONNECTED && cntr>0) {
    delay(200);
#ifdef IR_OLED_MENU
    d_clear();
    d_text(0, 0, VERSION);
    d_text(0, 1, "connecting ... "+String(cntr));
    d_text(0, 2, wifi_ap);    
    d_show();
#endif
    cntr--;
    //xprintf(PSTR("."));
    //Serial.print(".");
  }

  //WiFi.setAutoReconnect(true);
  //WiFi.persistent(true);

}
bool isSTA;
void reset_factory(){
  reset_eeprom();
  SPIFFS.remove("/custom.ini");
  readconfigs();
}
void setupwifi(int num)
{
  NOINTS

  while (1) {
    xprintf(PSTR("Wifi Initialization\n"));
    if (num) {
      //xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]) );
      //xprintf(PSTR("Disconnect\n"));
      WiFi.disconnect();
      server.close();
      //servertcp.close();
#ifdef WEBSOCKSERVER
      webSocket.close();
#endif
    }
    if (wifi_ap) connectWifi(0);
    String ipa = "->"+wifi_ap;
    ISWIFIOK = 0;
    if ((connect_timeout>0 && WiFi.status()!=WL_CONNECTED) || wifi_ap.length()==0) {
      xprintf(PSTR("Access Point Mode\n"));
      /*IPAddress local_IP(192, 168, 4, 1);
        IPAddress gateway(192, 168, 4, 9);
        IPAddress subnet(255, 255, 255, 0);
        WiFi.softAPConfig(local_IP, gateway, subnet);
      */
      WiFi.mode(WIFI_AP);
      ISWIFIOK = 0;

      if (wifi_dns=="")wifi_dns="ESP_CNC";  
      WiFi.softAP(wifi_dns);

      //ip = WiFi.softAPIP();
      //xprintf(PSTR("AP:%s Ip:%d.%d.%d.%d\n"), ((' '<wifi_dns[0]<'Z')?wifi_dns:ssid), fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]));
      ipa = wifi_dns + " : 4.1";
    }
        
#ifdef IR_OLED_MENU
    extern String IPA;
    IPA = ipa;
    Serial.end();
    hasSerial = 0;
#endif

    server.on("/setconfig", HTTP_GET, []() { // if the client requests the upload page
      NOINTS
      wifi_dns=server.arg("name");
      wifi_ap=server.arg("ap");
      wifi_pwd=server.arg("pw");
      server.send(200, "text/html", "OK");
      saveconfigs();
      delay(1000);
      ESP.restart();
      INTS
    });

    server.on("/gcode", HTTP_GET, []() { // if the client requests the upload page
      if (!uncompress) {
        char gc[100];
        server.arg("t").toCharArray(gc, 99);
        for (int i = 0; i < strlen(gc); i++) {
          buf_push(wf, gc[i]);
        }
        buf_push(wf, '\n');
        server.send(200, "text/html", "OK");
      }
      else {
        server.send(200, "text/plain", "ERR");
      }
    });
    server.on("/speed", HTTP_GET, []() { // if the client requests the upload page
      extern uint8_t head, tail, tailok;
      char gc[10];
      server.arg("t").toCharArray(gc, 9);
      extern float f_multiplier;
      f_multiplier = atoi(gc) * 0.01;
      tailok = tail + 5; // need to replanning all moves
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
        beginuncompress("/gcode.gcode",false,0);
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
        jobname=server.arg("jobname");
        beginuncompress(jobname,false,0);
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
        jobname=server.arg("jobname");
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
    server.on("/getfree",HTTP_GET,[](){
      if (!uncompress) {
        FSInfo fs_info;
        SPIFFS.info(fs_info);
        int tBytes = fs_info.totalBytes; 
        int uBytes = fs_info.usedBytes;
        server.send(200, "text/plain", "["+String(tBytes)+","+String(uBytes)+"]");
      }
    });
    server.on("/clearjobs", HTTP_GET, []() {
      NOINTS
        Dir dir = SPIFFS.openDir("/");
        while (dir.next()) {
          String s = dir.fileName();
          if (s.endsWith(".gcode") || s.endsWith(".jpg")) {
            SPIFFS.remove(s);  
          }
        }
        server.send(200, "text/plain", "Ok");
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
            if (str.length() > 2)
              str += ",";
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
        serverIP=server.client().remoteIP();
        server.send(200,"text/plain","OK");
        sendTelegram("test");
    });
 
    server.on("/upload", HTTP_GET, []() { // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
    serverIP=server.client().remoteIP();
    //serverIP=String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]);
      
      server.send(200, "text/html", "<form method=\"post\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"name\"> <input class=\"button\" type=\"submit\" value=\"Upload\"></form>");
    });
    server.on("/delete", HTTP_GET, []() { // if the client requests the upload page
      //xprintf(PSTR("Handle UPLOAD \n"));
      String s=server.arg("fn");
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
      server.sendHeader("Access-Control-Expose-Headers", "Access-Control-*");
      server.sendHeader("Access-Control-Allow-Headers", "Access-Control-*, Origin, X-Requested-With, Content-Type, Accept");
      server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS, HEAD");
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.send(200);
    }, // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload // Receive and save the file
             );

    server.onNotFound([]() { // If the client requests any URI
      if (!handleFileRead(server.uri())) // send it if it exists
        server.send(404, "text/plain", "404: Not Found ?"); // otherwise, respond with a 404 (Not Found) error
    });
    server.begin();

    //if (MDNS.begin(wifi_ap)) {
      //Serial.println("MDNS responder started");
    //}

#ifdef TCPSERVER
    servertcp.begin();
#endif
#ifdef WEBSOCKSERVER
    webSocket.begin(); // start the websocket server
    webSocket.onEvent(webSocketEvent); // if there's an incomming websocket message, go to function 'webSocketEvent'
#endif

    if (num)
      return;
    loadmeshleveling();

    touchserver(1, String(wifi_dns));
#ifdef DISABLESERIAL
    //Serial.end();
#endif


  if (ip[2]==1){
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
extern uint32_t cm;

uint32_t i_l_c=0;

void important_loop(){
  if (cm - wmc < (uncompress?1000000:200000)) {
    return;
  }
  IR_loop(0);
  //server.handleClient();
	/*
  motionloop();
    motionloop();
 #ifdef WEBSOCKSERVER 
  if (!uncompress) webSocket.loop();
motionloop();
#endif //webserver
  */
  wmc = cm;
}
long ipcheck=0;
void wifi_loop()
{
  extern int nextok;    
  // if motor idle  
  if ( nextok==0 && WiFi.status()==WL_CONNECTED && (ISWIFIOK==0 || (!uncompress && (cm - ipcheck > 10000000)))) {
      ip = WiFi.localIP();
      ISWIFIOK = 1;
      ipcheck=cm;
      extern String IPA;
      String OldIPA;
      OldIPA=IPA;
      //xprintf(PSTR("Connected to:%s Ip:%d.%d.%d.%d\n"), wifi_ap, fi(ip[0]), fi(ip[1]), fi(ip[2]), fi(ip[3]));

      //xprintf(PSTR("HTTP server started\n"));
      IPA = "Wifi : " + String(ip[2]);
      IPA += ".";
      IPA += String(ip[3]);
      if (OldIPA!=IPA)sendTelegram(wifi_dns+" SIAP BOS !! IPKU:"+String(ip[2])+"."+String(ip[3]));
  }
    
    
  if (cm - wmc < (uncompress?1000000:200000)) {
    return;
  }

  server.handleClient();
    motionloop();
    //MDNS.update();
    motionloop();
#ifdef WEBSOCKSERVER
  if (!uncompress)webSocket.loop();
    motionloop();
#endif //webserver
#ifdef USEOTA
  if (!uncompress) ArduinoOTA.handle();
    motionloop();
#endif

#ifdef TOUCHSERVER
  if (sendwait == 1)
    touchserver(0, String(wifi_dns));
#endif // touch
  wmc = cm;
  if (ISWIFIOK) {
    if (WiFi.status() != WL_CONNECTED) {
      //connectWifi();
      //return;
    }
  }
#ifdef TCPSERVER
  // wait for a new client:

  if (servertcp.hasClient()) {
    if (client)
      client.stop();
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
    }
    motionloop();
    else
      client.stop();
  }
#endif // tcp server
}
#endif

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
char gcode_loop()
{

#ifndef ISPC


  char c = 0;
  if (waitexecute) {
    if (tryexecute()) {
      ack_waiting = 1;
      if (ack_waiting) {
        //zprintf(PSTR("ok\n"));
        ack_waiting = 0;
      }
#ifdef timingG
      zprintf(PSTR("Gcode:%dus\n"), fi(micros() - gt));
#endif
    }
  } else
  {
    if (ack_waiting) {
      //zprintf(PSTR("ok\n"));
      ack_waiting = 0;
      n = 1;
    }
    // wifi are first class
#ifdef WIFISERVER
    if (buf_canread(wf)) {
      buf_pop(wf, c);
    }
    else
#endif
    {
      if (serialav()) {
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
      }
      else {
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
      gcode_parse_char(c);
    }
    //motionloop();
  }
#else
#endif
  return c;
}
int setupok = 0;

void setupother()
{

#ifdef ESP32
    SPIFFS.begin(true);
#else
    SPIFFS.begin();
#endif
/*
  if (!SPIFFS.exists("/config.ini")){
    // lets make one
    File f=SPIFFS.open("/config.ini","wb");
    for (int i=0;i<sizeof(web_configini);i++)
        f.write(web_configini[i]);
    f.close();
  }  
*/

  // important for KARYACNC to know the scale
  //zprintf(PSTR("EPR:3 185 %f Lscale\n"), ff(Lscale));
  readconfigs();
  //reload_eeprom();

#ifdef WIFISERVER
  setupwifi(0);
#endif

  init_gcode();
  timer_init();
  setupok = 1;
  zprintf(PSTR("start\nok\n"));
}
uint32_t t1;
void setup()
{
  // lets put all on INPUT_PULLUP
  
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  pinMode(RX, INPUT_PULLUP);
  pinMode(TX, INPUT_PULLUP);
  //pinMode(A0, INPUT_PULLDOWN);
  
  // put your setup code here, to run once:
  //  Serial.setDebugOutput(true);
  eeprominit;
  t1 = millis();
  //while (!Serial.available())continue;
  setupother();
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
    char c = 0;
    if (!uncompress)c = gcode_loop();
    motionloop();
    if (c == 0) uncompress_loop();
    motionloop();
    IR_loop(0);
    motionloop();
    servo_loop();
#ifdef WIFISERVER
    wifi_loop();
    motionloop();
#endif
}
