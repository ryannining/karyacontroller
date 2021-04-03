


/*

  G table
  0 - 0
  1 - 1
  2 - 28
  3 - 92

  M table
  0 - 3
  1 - 109 (104 is ignored)


  Compact representation of Gcode
  Header Bit
  0 = G/M
  1,2 = 8 gcode ID see table Gcode
  3,4,5,6,7 = f,x,y,z,e, if all zero then it will be S
  DATA byte
  F is 1 byte
  X,Y,Z,E is 2 byte
  S is 1 byte


  I think E is need special treatment because its must be precission to 5 digits

*/

#if defined(ESP8266) || defined(ESP32) || defined(SDCARD_CS)
#define fdebug

#include <WiFiClient.h>


#ifdef ESP32
#include <WiFi.h>
#include <SPIFFS.h>   // Include the SPIFFS library
#elif ESP8266
#include <ESP8266WiFi.h>
#endif


#include <FS.h>   // Include the SPIFFS library
#include "gcode.h"   // Include the SPIFFS library
#include "eprom.h"   // Include the SPIFFS library
#include "common.h"   // Include the SPIFFS library
#include "timer.h"   // Include the SPIFFS library
#ifdef IR_OLED_MENU
#include "ir_oled.h"
#endif
#define NOINTS timerPause();
#define INTS timerResume();

File fsGcode;
extern GCODE_COMMAND next_target;
int cntg28;
long lastjobt=0;
long lastjobt0=0;
int compressing = 0;
int uncompress = 0;
float startfrom_X = 0, startfrom_Y = 0;
void checkuncompress() {
  fsGcode = SPIFFS.open("/gcode.gcode", "r");
  if (!fsGcode) {
    zprintf(PSTR("File not found\n"));
    return;
  }
  zprintf(PSTR("Gcode %d bytes\n"), fi(fsGcode.size()));
  fsGcode.close();
}

#ifdef compression
void begincompress(String fn) {
  if (uncompress)return;
  fsGcode = SPIFFS.open(fn, "w");
  if (!fsGcode) {
    zprintf(PSTR("Cant open file\n"));
  } else {
    /*    byte wr=10;
        fsGcode.write((uint8_t *)&wr,1);
        int x=20;
        fsGcode.write((uint8_t *)&x,2);
    */
    compressing = 1;
    cntg28 = 2;
    zprintf(PSTR("Begin Compress Gcode\n"));
  }
}
void endcompress() {
  if (!compressing)return;
  /*
    byte h = 1;
    h |= 2 << 1;
    zprintf(PSTR("End Compress Gcode\n"));
    fsGcode.write((uint8_t *)&h, 1);
    fsGcode.close();
  */
  compressing = 0;
}

void addgcode()
{
  if (!compressing)return;
  /*  char h = 0;
    byte f, s;
    int x;
    if (next_target.seen_M) {
      h = 1;
      switch (next_target.M) {
        case 3: h |= 0; break;
        case 109: h |= 1 << 1; break;
        case 2: h |= 2 << 1; break; // final
        default: return; // not implemented
      }
      s = next_target.S;
      h |= 1 << 3;
      fsGcode.write((uint8_t *)&h, 1);
      fsGcode.write((uint8_t *)&s, 1);
      //zprintf(PSTR("M%d S%d\n"), fi(next_target.M), fi(s));
    }
    if (next_target.seen_G) {
      h = 0;
      switch (next_target.G) {
        case 0: h |= 0; break;
        case 1: h |= 1 << 1; break;
        case 28:
          h |= 2 << 1;
          cntg28--;
          break;
        case 92: h |= 3 << 1; break;
        default: return; // not implemented
      }
      if (next_target.seen_F)h |= 1 << 3;
      if (next_target.seen_X)h |= 1 << 4;
      if (next_target.seen_Y)h |= 1 << 5;
      if (next_target.seen_Z)h |= 1 << 6;
      if (next_target.seen_E)h |= 1 << 7;

      fsGcode.write((uint8_t *)&h, 1);
      //zprintf(PSTR("H%d G%d "), fi(h), fi(next_target.G));
      // write the parameter if exist
      if (next_target.seen_F) {
        f = fmin(next_target.target.F, 250);
        fsGcode.write((uint8_t *)&f, 1);
        //zprintf(PSTR("F%d "), fi(f));
      }
      if (next_target.seen_X) {
        x = (next_target.target.axis[nX] + 160) * 100;
        fsGcode.write((uint8_t *)&x, 2);
        //zprintf(PSTR("X%d "), fi(x));
      }
      if (next_target.seen_Y) {
        x = (next_target.target.axis[nY] + 160) * 100;
        fsGcode.write((uint8_t *)&x, 2);
        //zprintf(PSTR("Y%d "), fi(x));
      }
      if (next_target.seen_Z) {
        x = (next_target.target.axis[nZ] + 50) * 100;
        fsGcode.write((uint8_t *)&x, 2);
        //zprintf(PSTR("Z%d "), fi(x));
      }
      if (next_target.seen_E) {
        x = next_target.target.axis[nE] * 1000;
        fsGcode.write((uint8_t *)&x, 3);
        //zprintf(PSTR("E%d "), fi(x));
      }
    }
    if (!cntg28) {
      endcompress();
      cntg28 = 2;
    }
    //zprintf(PSTR("\n"));
  */
}
int compress_loop() {
  if (uncompress)return 0;
  if (next_target.seen_M) {
    switch (next_target.M) {
      /*case 130: checkuncompress(); goto ret;
        case 131: begincompress(); goto ret;
        case 132: endcompress(); goto ret;
        case 133: beginuncompress(); goto ret;
        case 134: enduncompress(); goto ret;*/
      case 135: SPIFFS.format(); goto ret;
    }
  }
  /*
    if (compressing && (next_target.seen_M || next_target.seen_G)) {
    if (next_target.seen_G && (next_target.G == 90)) {
      next_target.option_all_relative = 0;
      zprintf(PSTR("Abs\n"));
    } else if (next_target.seen_G && (next_target.G == 91)) {
      next_target.option_all_relative = 1;
      zprintf(PSTR("Rel\n"));
    }
    addgcode();
    return 1;
    }*/
ret:
  return 0;
}
#else
int compress_loop() {
  if (uncompress)return 0;
  if (next_target.seen_M) {
    switch (next_target.M) {
      /*case 130: checkuncompress(); goto ret;
        case 131: begincompress(); goto ret;
        case 132: endcompress(); goto ret;
        case 133: beginuncompress(); goto ret;
        case 134: enduncompress(); goto ret;*/
      case 135: SPIFFS.format(); break;
    }
  }
  return 0;
}

#endif

void beginuncompress(String fn);
void enduncompress();
void deletejob(String fn) {
        SPIFFS.remove(fn);
        SPIFFS.remove(fn + ".jpg");
}
// ============================================= uncompress ===============================================
bool dummy_uncompress=false;
int uctr = 0;
long eE = 0;
int gxver = 1;
int xySize, zSize, eSize, xyLimit, zLimit, eLimit;
float xyScale, zScale, eScale, fScale, AX, AY, AZ, AE;
int mediaX, mediaY;

#define datasize(n) ((1 << (n*8-1))-2)
int gcodesize, gcodepos;
void beginuncompress(String fn) {
  if (uncompress)return;
  mediaX = 0;
  mediaY = 0;
  uctr = 1;
  eE = 0;
  lastjobt0=millis();
  
  fsGcode = SPIFFS.open(fn, "r");
  if (!fsGcode) {
    zprintf(PSTR("File not found\n"));
    return;
  }
  gcodesize = fsGcode.size();
  gcodepos = 3;
  char hdr[3];
  fsGcode.read((uint8_t *)&hdr, 3);
  if (hdr[0] == 'G' && hdr[1] == 'X') {
    gxver = hdr[2];
  } else {
    zprintf(PSTR("File is not GX\n"));
    fsGcode.close();
    return;
  }

  if (gxver == 1) {
    xyScale = 1 / 0.005;
    zScale = 1 / 0.1;
    eScale = 1 / 0.02;
    xySize = 2;
    zSize = 1;
    eSize = 1;
    fScale = 2;

  } else {
    /*    xyScale=1/0.005;
        zScale=1/0.1;
        eScale=1/0.02;
        xySize=2;
        zSize=1;
        eSize=1;
        fScale=2;
    */
    zprintf(PSTR("Unknown GX version %d\n"), fi(gxver));
    fsGcode.close();
    return;
  }
  xyLimit = datasize(xySize);
  zLimit = datasize(zSize);
  eLimit = datasize(eSize);
  AX = cx1;
  AY = cy1;
  AZ = ocz1;
  AE = 0;

  uncompress = 1;
  cntg28 = 2;
  zprintf(PSTR("Begin Uncompress Gcode %d bytes\n"), fi(fsGcode.size()));
}
void enduncompress() {
  if (!uncompress)return;
  fsGcode.close();
  uncompress = 0;
  zprintf(PSTR("End Uncompress Gcode\n"));
  waitbufferempty();
  lastjobt=millis()-lastjobt0;
  #ifdef IR_OLED_MENU
  xdisplay.Reset();
  #endif  
}
int ispause = 0;

void gcodereadfile(uint8_t * addr, int len) {

}

long buffidx = 0;
int buffpage = 0;
char buffers[10000];
int connected = 0;
#define IOT_IP_ADDRESS "172.245.97.171"
int gid = 0;
void realreadnet(int s, int l, int bi) {
  if (WiFi.status() != WL_CONNECTED)return;
  //wifiConnect();
  WiFiClient client;
  client.connect(IOT_IP_ADDRESS, 80);
  if (client.connected()) {

    client.print("GET /download?gid=");
    client.print(String(gid)); client.print("&start="); client.print(String(s));
    client.print("&length="); client.print(String(l));
    client.print(" HTTP/1.1\r\nHost: "); client.print( String(IOT_IP_ADDRESS)); client.print("\r\nContent-Length: 0\r\n\r\n");

    delay(200);
    char lc, c;
    int cp = 0;
    char cc[20];
    // skip all header
    while (client.available()) {
      c = client.read();
      if ((c == '>') && (lc == '<'))break;
      lc = c;
    }
    // read data
    while (1) {
      if (client.available()) {
        c = client.read();
        if (c == 10) break;
        buffers[bi] = c; bi++;
        if ((c == '>') && (lc == '<'))break;
        lc = c;
      } else delay(100);
    }
  }
}
void gcodereadnet(uint8_t * addr, int len) {
  // read
  // check if is need other read
  if (buffidx > 1000) {

  }
}

float dMax,fMax,lF,lF0,lF1,xMin,xMax,yMin,yMax,zMin,zMax,tMax;
int sMax;
byte repeatheader;
byte repeatsame;
// version, old is using eSize 1
void uncompressaline() {
  if (ispause)return;
  byte h;
  byte s;
  int32_t x = 0;
  if (!fsGcode.available()) {
    enduncompress();
    return;
  }  
  if (repeatheader==0){
	fsGcode.read((uint8_t *)&h, 1); gcodepos++;
	if (repeatsame) {
		repeatheader=h;
	}
  }	else {
	h=repeatheader;
	repeatsame--;
	if (repeatsame==0)repeatheader=0;
  }
  uctr++;
  //if (uctr>10) return enduncompress();
  if (h & 1) {
    next_target.seen_M = 1;
    switch ((h >> 1) & 3) {
      case 0: next_target.M = 3; break;
      case 1: next_target.M = 109; break;
      case 2: 
		// special  for laser, this will inform how many data with same header but will flip and flop G0 G1
		s = 0;
		fsGcode.read((uint8_t *)&s, 1); gcodepos++;
		repeatsame=s;
		repeatheader=0;
      return; break;
    }
    //zprintf(PSTR("%d H%d M%d S%d\n"), fi(uctr), fi(h), fi(next_target.M), fi(s));
  } else {
    next_target.seen_G = 1;
    switch ((h >> 1) & 3) {
      case 0: next_target.G = 0; break;
      case 1: next_target.G = 1; break;
      case 2:
        next_target.G = 28;
        cntg28--;
        break;
      case 3: next_target.G = 92; eE = 0; break;
    }
  }
  // read the parameter
  //zprintf(PSTR("%d H%d G%d "), fi(uctr), fi(h), fi(next_target.G));
  if (h & (1 << 3)) {
    s = 0;
    fsGcode.read((uint8_t *)&s, 1); gcodepos++;
    // F and S is same in M code
    if (next_target.seen_M) // if M then its S
    {
      next_target.seen_S = 1;
      next_target.S = s;
      sMax=fmax(sMax,s);
    } else { //else its F
      next_target.seen_F = 1;
      lF=next_target.target.F = s * fScale;
      if (next_target.G==1){
          lF1=lF;
      }else lF0=lF;
      
    }

    //zprintf(PSTR("F%d "), fi(s));
  }
  lF=(next_target.G==1)?lF1:lF0;

  
    float D=0;
    float d;

  if (h & (1 << 4)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, xySize); gcodepos += xySize;
    //zprintf(PSTR("X%d "), fi(x));
    next_target.seen_X = 1;
    if (next_target.seen_M) {
      x = float(x - xyLimit) * 100 / xyScale;
      mediaX = x;
      next_target.target.axis[nX] = x;
    } else {
      d=float(x - xyLimit) / xyScale;
      D+=d*d;
      AX += d;
      next_target.target.axis[nX] = AX;
      //next_target.target.axis[nX] *= xyscale;
    }
  }
  if (h & (1 << 5)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, xySize); gcodepos += xySize;
    next_target.seen_Y = 1;
    if (next_target.seen_M) {
      x = float(x - xyLimit) * 100 / xyScale;
      mediaY = x;
      next_target.target.axis[nY] = x;
    } else {
      //zprintf(PSTR("Y%d "), fi(x));
      d=float(x - xyLimit) / xyScale;
      D+=d*d;
      AY += d;
      next_target.target.axis[nY] = AY;
    }
    //next_target.target.axis[nY] *= xyscale;
  }
  if (h & (1 << 6)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, zSize);gcodepos+=xySize;
    //zprintf(PSTR("Z%d "), fi(x));
    next_target.seen_Z = 1;
    d=float(x - zLimit) / zScale;
    D+=d*d;
    AZ += d;
    next_target.target.axis[nZ] = AZ;
  }
  if (h & (1 << 7)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, eSize);gcodepos+=eSize;
    
    if (next_target.seen_M) {
		//zprintf(PSTR("E%d "), fi(x));
		next_target.seen_P = 1;
		next_target.P = x*100;
    } else {
		next_target.seen_E = 1;
		AE += float(x - eLimit) / eScale;
		next_target.target.axis[nE] = AE;
	}
  } 
  //zprintf(PSTR("\n"));

  if (!dummy_uncompress)process_gcode_command();
  else {
        xMin=fmin(xMin,AX);
        xMax=fmax(xMax,AX);
        yMin=fmin(yMin,AY);
        yMax=fmax(yMax,AY);        
        zMin=fmin(zMin,AZ);
        zMax=fmax(zMax,AZ);
        if (AZ<=0 && D>0){
            D=sqrt(D);
            dMax+=D;
            tMax+=D/lF;
        }
  }  
  lastjobt=millis()-lastjobt0;
  reset_command();
  if (!cntg28) {
    enduncompress();
    cntg28 = 2;
  }
}

void dummy_beginuncompress(String fn){
    dummy_uncompress=true;
    zMin=xMin=yMin=10000;
    zMax=xMax=yMax=-10000;
    dMax=lF=lF0=lF1=sMax=fMax=0;
    tMax=0;
    
    beginuncompress(fn);
    while (uncompress){
        uncompressaline();
        
    }
    enduncompress();
    dummy_uncompress=false;
    
}


// ========================== LOOP =============================
void uncompress_loop() {
  if (uncompress) {
    // do the uncompress job
    uncompressaline();
  }
}

#endif
