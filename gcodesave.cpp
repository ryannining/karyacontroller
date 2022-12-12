


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

#if defined(ESP8266) || defined(ESP32) 
#define fdebug

#include <WiFiClient.h>


#ifdef ESP32
#include <WiFi.h>
#include <SPIFFS.h>   // Include the SPIFFS library
#elif ESP8266
#include <ESP8266WiFi.h>
#endif


#include <FS.h>   // Include the SPIFFS library
#include "common.h"   // Include the SPIFFS library
#include "gcode.h"   // Include the SPIFFS library
#include "eprom.h"   // Include the SPIFFS library
#include "timer.h"   // Include the SPIFFS library
#include "gcodesave.h"   // Include the SPIFFS library

#ifdef IR_OLED_MENU
#include "ir_oled.h"
#endif

#define NOINTS timerPause();
#define INTS timerResume();


int shapes_ctr;
tshapes shapes[SHAPESNUM+1] __attribute__ ((aligned (4)));
float PZ=0;
int lastSv=0;
bool singleobj;

File fsGcode;
extern GCODE_COMMAND next_target;
int cntg28;
long lastjobt = 0;
long lastjobt0 = 0;
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

void beginuncompress(String fn,bool resume,int pos);
void enduncompress(bool force=false);
void deletejob(String fn) {
  SPIFFS.remove(fn);
  SPIFFS.remove(fn + ".jpg");
}
// ============================================= uncompress ===============================================
bool dummy_uncompress = false;
int uctr = 0;
long eE = 0;
int gxver = 1;
int xySize, zSize, eSize, xyLimit, zLimit, eLimit;
float xyScale, zScale, eScale, fScale, AX, AY, AZ, AE,OAX,OAY,OAZ;
int mediaX, mediaY;

#define datasize(n) ((1 << (n*8-1))-2)
int gcodesize, gcodepos;
String fjob;
float dMax, fMax, lF, lF0, lF1, xMin, xMax, yMin, yMax, zMin, zMax, tMax;
int sMax;
byte repeatheader;
byte repeatsame;

void prepareposition(){
	  OAX=cx1;
	  OAY=cy1;
	  OAZ=ocz1;
	  AX = 0;
	  AY = 0;
	  AZ = 0;
	  AE = 0;
}
long realstart;
void beginuncompress(String fn,bool resume,int pos) {
  if (uncompress)return;
  mediaX = 0;
  mediaY = 0;
  uctr = 1;
  eE = 0;
  lastjobt0 = millis();

  fsGcode = SPIFFS.open(fn, "r");
  fjob=fn;
  if (!fsGcode) {
    zprintf(PSTR("File not found\n"));
    return;
  }
  gcodesize = fsGcode.size();

  char hdr[3];
  fsGcode.read((uint8_t *)&hdr, 3);
  if (hdr[0] == 'G' && hdr[1] == 'X') {
    gxver = hdr[2];
  } else {
    zprintf(PSTR("File is not GX\n"));
    fsGcode.close();
    return;
  }
  fScale=2;
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
  

  ///*
  if (!resume) {
	prepareposition();
	singleobj=false;
	realstart=0;
	extern bool makezeropoint;
	set_tool(255);
	if (!dummy_uncompress){
		if (makezeropoint){
			addmove(5, 0, 0, -1, 0, 0, 1); // zero point
		}	
		addmove(100, 0, 0, 5, 0, 1, 1); // zero point
	}
  } else {
	//
	uint32_t pp=shapes[pos].pos;
	
	reset_command();
	realstart=gcodepos=pp & ((1<<24)-1);
	lF1=lF0=lF=next_target.target.F=(pp>>24);
	next_target.seen_F=1;
	//prepareposition();
	AX=0;
	AY=0;
	AZ=0;

	//fsGcode.seek(gcodepos);
    set_tool(0);
    set_tool(255);
  }
  //*/
  uncompress = 1;
  //repeatheader=0;
  cntg28 = 2;
  zprintf(PSTR("Begin Uncompress Gcode %d bytes\n"), fi(fsGcode.size()));
}

void enduncompress(bool force) {
  if (uncompress!=1)return;
  fsGcode.close();
  uncompress = 2;

}
int ispause = 0;

void gcodereadfile(uint8_t * addr, int len) {

}

int dVfunc(int s){
	if (s<51)return s;
	if (s<121)return (50+(s-50)*2);
	return (190+(s-120)*4);
}
int eVfunc(int s){
	if (s<51)return s;
	if (s<191)return (s+50)/2;
	return (s+290)/4;
}

long buffidx = 0;
int buffpage = 0;
char buffers[10000];
int connected = 0;
#define IOT_IP_ADDRESS "172.245.97.171"
int gid = 0;
void realreadnet(int s, int l, int bi) {
	/*
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
  */
}
void gcodereadnet(uint8_t * addr, int len) {
  // read
  // check if is need other read
  if (buffidx > 1000) {

  }
}


// version, old is using eSize 1

void uncompressaline() {
  //if (ispause)return;
  byte h;
  byte s;
  int32_t x = 0;
  if (!fsGcode.available()) {
    enduncompress();
    // auto remove file if start with _
    if (fjob.startsWith("/_")){
		SPIFFS.remove(fjob);
	}
    return;
  }
  int qpos=gcodepos=fsGcode.position();
  fsGcode.read((uint8_t *)&h, 1);
  /*
  if (repeatheader == 0) {
    fsGcode.read((uint8_t *)&h, 1); 
    if (repeatsame) {
      repeatheader = h;
    }
  }	else {
    h = repeatheader;
    repeatsame--;
    if (repeatsame == 0)repeatheader = 0;
  }*/
  
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
        fsGcode.read((uint8_t *)&s, 1); 
        repeatsame = s;
        repeatheader = 0;
        return; break;
    }
    //zprintf(PSTR("%d H%d M%d S%d\n"), fi(uctr), fi(h), fi(next_target.M), fi(s));
  } else {
    next_target.seen_G = 1;
    switch ((h >> 1) & 3) {
      case 0: next_target.G = 0; break;
      case 1: 
		next_target.G = 1; 
		// if mode CNC check tool
		extern int lasermode,lastS;
		if (lasermode==0 && lastS<30) set_tool(255); 

		break;
      case 2:
        next_target.G = 28;
        //cntg28--;
        break;
      case 3: next_target.G = 92; eE = 0; break;
    }
  }
  
  if (dummy_uncompress && next_target.seen_G && next_target.G<=1 && AZ>0){
    int q=shapes_ctr;
    shapes[q].pos=qpos | (int(lF)<<24);
    shapes[q].px=AX;
    shapes[q].py=AY;
    shapes[q].pz=AZ;
    PZ=AZ;
  } else if (singleobj && AZ<=0){
	PZ=AZ;
  }
  
  // read the parameter
  // zprintf(PSTR("%d H%d G%d "), fi(uctr), fi(h), fi(next_target.G));
  if (h & (1 << 3)) { //F or S
    s = 0;
    fsGcode.read((uint8_t *)&s, 1); 
    // F and S is same in M code
    if (next_target.seen_M) // if M then its S
    {
      next_target.seen_S = 1;
      next_target.S = s;
      sMax = fmax(sMax, s);
      lastSv=s;
    } else { //else its F
      next_target.seen_F = 1;
      lF = next_target.target.F = s*fScale;
      if (next_target.G == 1) {
        lF1 = lF;
      } else lF0 = lF;

    }

    //zprintf(PSTR("F%d "), fi(s));
  }
  lF = (next_target.G == 1) ? lF1 : lF0;
  if (lF<3)lF=3;

  float D = 0;
  float d;

  if (h & (1 << 4)) { // X
    x = 0;
    fsGcode.read((uint8_t *)&x, xySize); 
    //zprintf(PSTR("X%d "), fi(x));
    next_target.seen_X = 1;
    if (next_target.seen_M) {
      x = float(x - xyLimit) * 100 / xyScale;
      mediaX = x;
      next_target.target.axis[nX] = x;
    } else {
      d = float(x - xyLimit) / xyScale;
      D += d * d;
      AX += d;
      next_target.target.axis[nX] = AX+OAX;
      //next_target.target.axis[nX] *= xyscale;
    }
  }
  if (h & (1 << 5)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, xySize); 
    next_target.seen_Y = 1;
    if (next_target.seen_M) {
      x = float(x - xyLimit) * 100 / xyScale;
      mediaY = x;
      next_target.target.axis[nY] = x;
    } else {
      //zprintf(PSTR("Y%d "), fi(x));
      d = float(x - xyLimit) / xyScale;
      D += d * d;
      AY += d;
      next_target.target.axis[nY] = AY+OAY;
    }
    //next_target.target.axis[nY] *= xyscale;
  }
  if (h & (1 << 6)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, zSize);
    //zprintf(PSTR("Z%d "), fi(x));
    next_target.seen_Z = 1;
    d = float(x - zLimit) / zScale;
    if (next_target.G == 28) {
      // G28 Z1 -> probe Z and set Z 1 after success
      next_target.target.axis[nZ] = d;
    } else {
      D += d * d;
      AZ += d;
      
      if (dummy_uncompress && AZ<=0 && PZ>0){ // detect plunge and add to shape database
		  if (shapes_ctr<SHAPESNUM)shapes_ctr++;
		  PZ=AZ;
	  } else if (singleobj && AZ>0 && PZ<=0){
		// this is up, so its the last
		enduncompress();
		//waitbufferempty();
		addmove(100, 0, 0, 10, 0, 1, 1);
		//addmove(100, cx1, cy1, OAZ, 0, 1, 0);		
		addmove(100, OAX, OAY, ocz1, 0, 1, 0);
		addmove(100, 0, 0, -10, 0, 1, 1);
		//waitbufferempty();		
		return;
	  }
	  
      next_target.target.axis[nZ] = AZ+OAZ;
    }
  }
  if (h & (1 << 7)) {
    x = 0;
    fsGcode.read((uint8_t *)&x, eSize);

    if (next_target.seen_M) {
      //zprintf(PSTR("E%d "), fi(x));
      next_target.seen_P = 1;
      next_target.P = x * 100;
    } else {
      next_target.seen_E = 1;
      AE += float(x - eLimit) / eScale;
      next_target.target.axis[nE] = AE ;
    }
  }
  //zprintf(PSTR("\n"));

  if (!dummy_uncompress && qpos>=realstart){
	  waitexecute=true;
	  //process_gcode_command();
  } else {
	waitexecute=false;

    xMin = min(xMin, AX);
    
    xMax = max(xMax, AX);
    
    yMin = min(yMin, AY);
    
    yMax = max(yMax, AY);
    
    zMin = min(zMin, AZ);
    
    zMax = max(zMax, AZ);
    
    if (AZ <= 0 && D > 0) {
      D = sqrt(D);
      dMax += D;
      tMax += D / lF;
    }
    if (realstart>0){
		extern float F0,F1;
		// handle Feedrate
		if (next_target.seen_F && next_target.seen_G) {
			if (next_target.G==0)F0 = next_target.target.F; else F1 = next_target.target.F;
		}
		// handle Spindle power
		if (next_target.seen_S) {
			set_tool(next_target.S);
		}
	}
    reset_command();
  }
  lastjobt = millis() - lastjobt0;
}

void dummy_beginuncompress(String fn) {
  dummy_uncompress = true;
  singleobj=false;
  shapes_ctr=0;
  PZ=0;
  zMin = xMin = yMin = 15000;
  zMax = xMax = yMax = -15000;
  dMax = lF = lF0 = lF1 = sMax = fMax = 0;
  tMax = 0;

  beginuncompress(fn,false,0);
  int fc=100;
  while (uncompress==1) {
    uncompressaline();
	//if (fc--<10){fc=100;feedthedog();}
  }
  enduncompress();
  uncompress=0;
  reset_command();
  waitexecute=false;
  dummy_uncompress = false;
  // let the min and max relative

}


// ========================== LOOP =============================
void uncompress_loop() {
  if (uncompress==1) {
	  
    // do the uncompress job
    for (int j=15;j>0;j--){
		tryexecute();
		if (uncompress==1 && !waitexecute && RUNNING && !PAUSE &&  nextbuff(head) != tail) uncompressaline();
		domotionloop
	}
  } else if (uncompress==2){
	if (head==tail && cmhead==cmtail) {
		uncompress=0;
	  //zprintf(PSTR("End Uncompress Gcode\n"));
	  //if (!force)waitbufferempty();
	  lastjobt = millis() - lastjobt0;
	#ifdef IR_OLED_MENU
	  extern void load_info();
	  load_info();
	  xdisplay.Reset();
	#endif
		extern String jobname;
		extern String uptime(long w);
		extern void sendTelegram(String stext);
		sendTelegram(wifi_dns+" finish job "+jobname+" Time:"+uptime(lastjobt));
	}
  }
}

#endif
