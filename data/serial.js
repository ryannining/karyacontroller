//console.log("App Running");

var lastw="";
var oktosend=1;
var connectionId = null;
var eline=0;
var okwait=1;
var running=0;
var px=0;var py=0;
function comconnect(){
  var bt=document.getElementById('btconnect');
	if (bt.innerHTML=="Connect"){
		connect(document.getElementById('comport').value);
		bt.innerHTML='Disconnect';
	} else {
		disconnect();
		bt.innerHTML='Connect';
	}
}
function testlaser(){
	sendgcode("M3 S300 P50");
	sendgcode("M3 S0");
}

function gcodemove(x,y){
	var mm=document.getElementById('move').value;
	px=px+x*mm;
	py=py+y*mm;
	sendgcode("G0 F5000 X"+(px)+" Y"+(py));
}
function gcoderight(){gcodemove(-1,0);}
function gcodeleft(){gcodemove(1,0);}
function gcodeup(){gcodemove(0,1);}
function gcodedown(){gcodemove(0,-1);}

function homing(){
	sendgcode("G28");
	px=0;
	py=0;
}
function setashome(){
   sendgcode("G0 Z0");
	sendgcode("G92 X0 Y0 Z0 E0");
	px=0;
	py=0;
}


var comtype=0; // 0 serial, 1 websocket

function sendgcode(g){
  if (comtype==0){
	  try {
		writeSerial(g+"\n");
	  } catch (e) {

	  }
  }
  if (comtype==1){
	ws.send(g+"\n");
  }
  okwait++;
}
function nextgcode(){
	if (okwait) return;
	if (!running)return;
	while(eline<egcodes.length){
		var g=egcodes[eline];
		eline++;
		if ((g) && (g[0]!=';')) {
			sendgcode(g.split(";")[0]);
			return;
		}
	}
  stopit();
}
function stopit(){
  var bt=document.getElementById('btexecute');
  bt.innerHTML="Execute";
  running=0;
  okwait=0;
  egcodes=[];
}
var egcodes=[];
function execute(gcodes){
	egcodes=gcodes.split("\n");
	eline=0;
	running=egcodes.length;
	okwait=0;
	nextgcode();
}
function executegcodes(){
  var bt=document.getElementById('btexecute');
  if (bt.innerHTML=="Execute") {
    execute (document.getElementById('gcode').value);
    bt.innerHTML="Stop";
  } else {
    stopit();
    sendgcode("M2");
  }
}
function executepgcodes(){execute(document.getElementById('pgcode').value);}
function executeicodes(){execute(document.getElementById('icode').value);}

function pause(){
  var bt=document.getElementById('btpause');
  if (bt.innerHTML=="PAUSE") {
    running=0;
    sendgcode("M3 S200 P10");
    bt.innerHTML="RESUME";
  } else {
    running=1;
    sendgcode("M3 S200");
    nextgcode();
    bt.innerHTML="PAUSE";
  }
}

var onReadCallback = function(s){
	//console.log(s);
	for (var i=0;i<s.length;i++){
		if ((s[i]=="\n") || (s[i]==" ")){
			if (lastw.toUpperCase()=="OK"){
				okwait--;
				nextgcode();
			}
			lastw="";
		} else lastw=lastw+s[i];
	}
}

var listPorts = function() {
	chrome.serial.getDevices(onGotDevices);
};


var onGotDevices = function(ports) {
  var s="";
	for (var i=0; i<ports.length; i++) {
    if (ports[i].path)s=s+"<option value="+ports[i].path+">"+ports[i].path+"</option>";
  }
	document.getElementById("comport").innerHTML=s;
};

var connect = function(path) {
	var options = {bitrate: 115200};
	chrome.serial.connect(path, options, onConnect)
};

var onConnect = function(connectionInfo) {
	//console.log(connectionInfo);
	connectionId = connectionInfo.connectionId;
};

var disconnect = function() {
	chrome.serial.disconnect(connectionId, onDisconnect);
};

var onDisconnect = function(result) {
  if (result) {
    //console.log("Disconnected from the serial port");
  } else {
    //console.log("Disconnect failed");
  }
};

//var setOnReadCallback = function(callback) {
//	onReadCallback = callback;
//	chrome.serial.onReceive.addListener(onReceiveCallback);
//};

var onReceiveCallback = function(info) {
    if (info.connectionId == connectionId && info.data) {
      var str = convertArrayBufferToString(info.data);
      //console.log(str);
      if (onReadCallback != null) {
      	onReadCallback(str);
      }
    }
 };
var onSend = function(sendInfo) {
  //console.log(sendInfo);
}

var writeSerial = function(str) {
  chrome.serial.send(connectionId, convertStringToArrayBuffer(str), onSend);
}
// Convert ArrayBuffer to string
var convertArrayBufferToString = function (buf) {
    return String.fromCharCode.apply(null, new Uint8Array(buf));
}
// Convert string to ArrayBuffer
var convertStringToArrayBuffer = function(str) {
  var buf=new ArrayBuffer(str.length);
  var bufView=new Uint8Array(buf);
  for (var i=0; i<str.length; i++) {
    bufView[i]=str.charCodeAt(i);
  }
  return buf;
}

function changematerial(){
  val=getvalue("material");
  val=val.split(",");
  setvalue("feed",val[0]);
  setvalue("repeat",val[1]);
  harga=val[2];
  if (val.length==4){
	document.getElementById("cmode").value=3;	
	setvalue("zdown",val[3]);
	setvalue("tabc",2);
  }
  if (val.length==3){
	document.getElementById("cmode").value=1;	
	setvalue("zdown",0);
	setvalue("tabc",0);
	
  }
	  
}
function modechange(){
  val=getvalue("cmode");
  if (val==1) {
    setvalue("pup","M3 S0");
    setvalue("pdn","M3 S160");
  }
  if (val==2) {
    setvalue("pup","M3 S0");
    setvalue("pdn","M3 S160");
  }
  if (val==3) {
    setvalue("pup","G0 Z2 F1000");
    setvalue("pdn","G0 Z=cncz F60");
    setvalue("feed","3");
  }
}
//onclick="setashome();"
function initserial(){
   try {
     chrome.serial.onReceive.removeListener(onReceiveCallback);
     chrome.serial.onReceive.addListener(onReceiveCallback);
     listPorts();
	 comtype=0;
   } catch (e) {
	 comtype=-1;
   }
}
setclick("btinitser",initserial);
setclick("btconnect",comconnect);
setclick("btsethome",setashome);
setclick("bthoming",homing);
setclick("btinit",executeicodes);
setclick("btpreview",executepgcodes);
setclick("btexecute",executegcodes);
setclick("btpause",pause);
setclick("bthit",testlaser);
setclick("btleft",gcodeleft);
setclick("btup",gcodeup);
setclick("btdn",gcodedown);
setclick("btright",gcoderight);
setclick("btrecode",refreshgcode);
setclick("btsend",function(){sendgcode(getvalue("edgcode"));})
setclick("btmotoroff",function(){sendgcode("M84");})
setevent("change","btfile",openFile);
setevent("change","cmode",modechange);

setevent("change","material",changematerial);

setclick("btcopy1",function(){copy_to_clipboard('gcode');});
setclick("btcopy2",function(){copy_to_clipboard('pgcode');});
var stotype=0;
try {
storage=chrome.storage.local;

} catch(e){
stotype=1;
storage=localStorage;	
}

function savesetting(){
   a=document.getElementsByClassName("saveit");
   sett={};
   for (var i=0;i<a.length;i++){
         sett[a[i].id]=a[i].value;
   }
   if (stotype==1){
     storage.setItem("settings",JSON.stringify(sett));
     storage.setItem("text1",text1);
   } else {
     storage.set({"settings":sett,"text1":text1});
   }
}
setclick("btsaveset",savesetting);


if (stotype==0){
	try {
	   storage.get("text1",function(r){text1=r.text1;})
	   storage.get("settings",function(r){
		  sett=r.settings;
		  for (var k in sett){
				setvalue(k,sett[k]);
		  }
		  refreshgcode();
	   });
	} catch (e) {
	}
} else {
   text1=storage.text1;
   if (text1==undefined)text1="";
   if (storage.settings!=undefined){
	   sett=JSON.parse(storage.settings);
	   for (var k in sett){ 
			setvalue(k,sett[k]);
	   }
   }
   if (text1)refreshgcode();
}	


// websocket
window.onload=function(){
	a=document.activeElement;
	if (a.style.opacity==0)a.hidden=true;
};
if (window.location.host){
	var lastcomtype=comtype;
	comtype=1;
	function handlemessage(m){
		msg=m.data;
		onReadCallback(msg);
		console.log(msg);
	}

	ws=new WebSocket('ws://'+window.location.host+':81/', ['arduino']);
	ws.onerror=function(e){comtype=lastcomtype;} // back to serial if error.
	ws.onmessage=handlemessage;
}