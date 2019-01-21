var wsconnected = 0;
var lastw = "";
var oktosend = 1;
var connectionId = null;
var eline = 0;
var okwait = 0;
var running = 0;
var px = 0;
var py = 0;
var pz = 0;
var pe = 0;
var etime=new Date();
var checktemp=1;

var comtype = 0; // 0 serial, 1 websocket
var egcodes = [];
var debugs = 2;
function sendgcode(g) {
    if (debugs & 1) console.log(g);
	try {
		if (comtype == 0) writeSerial(g + "\n");
		if (comtype == 1) ws.send(g + "\n");
	}
	catch (e) {        }
}

function nextgcode() {
    if (comtype == 1 && !wsconnected) {
        //setTimeout(nextgcode,1000);
        return;
    }; // wait until socket connected
    if (okwait>0) return;
    if (!running) return;
    while (eline < egcodes.length) {
        var g = egcodes[eline];
        eline++;
		if (g==";PAUSE")pause();
        if ((g) && (g[0] != ';') && (g[0] != '(')) {
			okwait=1;
            sendgcode(g.split(";")[0]);
            return;
        }
    }
    sendgcode("G4");
	sendgcode("M114");
    stopit();
}

function stopit() {
	var ms=etime.getTime();
	etime=new Date();
	console.log("Stop "+etime);
	ms=etime.getTime()-ms;
	mss="Real time:"+mround(ms/60000.0);
	console.log(mss);
	//$("infolain").innerHTML=mss;
    var bt = document.getElementById('btexecute');
    bt.innerHTML = "Print";
    var bt = document.getElementById('btpause');
	bt.innerHTML = "Pause";
    running = 0;
    okwait = 0;
    egcodes = [];
}

function execute(gcodes) {
	etime=new Date();
	console.log("Start "+etime);
    var bt = document.getElementById('btpause');
	bt.innerHTML = "PAUSE";
    egcodes = gcodes.split("\n");
    eline = 0;
    running = egcodes.length;
    okwait = 0;
    nextgcode();
    //sendgcode("M105");
}

function idleloop(){
	
	if (wsconnected && checktemp) {
        if (!running)sendgcode("M105");
		checktemp=0;
    }
	setTimeout(idleloop,3000);
}

var ss = "";
var eeprom = {};
var ineeprom = 0;
var eppos = 0;
var resp1="";

var onReadCallback = function(s) {
	resp1+=s;
    for (var i = 0; i < s.length; i++) {
        if (s[i] == "\n") {
			
			if (ss.indexOf("Z:")>0){
				px=parseFloat(ss.substr(ss.indexOf("X:")+2));
				py=parseFloat(ss.substr(ss.indexOf("Y:")+2));
				pz=parseFloat(ss.substr(ss.indexOf("Z:")+2));
				pe=parseFloat(ss.substr(ss.indexOf("E:")+2));
			}
            if (debugs & 2) console.log(ss);
            ss = "";
        } else ss += s[i];
        if ((s[i] == "\n") || (s[i] == " ") || (s[i] == "*")) {
            if (ineeprom > 0) {
                if (ineeprom == 3) eppos = lastw;
                if (ineeprom == 2) eeprom[eppos] = lastw;
                if (ineeprom == 1) {
                    var sel = document.getElementById("eepromid");
                    sel.innerHTML += "<option value=\"" + eeprom[eppos] + ":" + eppos + "\">" + lastw + "</option>";
                }
                ineeprom--;
            }
            if (lastw.toUpperCase().indexOf("EPR:") >= 0) {
                ineeprom = 3;
            }
            if (lastw.toUpperCase().indexOf("T:") >= 0) {
                document.getElementById("info3d").innerHTML = lastw;
				checktemp=1;
            }
			
			isok=(lastw.length==2) && (lastw[0].toUpperCase()=='O');
            if (isok || (lastw.toUpperCase().indexOf('OK')>=0)|| (lastw.toUpperCase().indexOf('WAIT')>=0)) {
                okwait=0;
                nextgcode();
            }
            lastw = "";
        } else lastw = lastw + s[i];
    }
}


function connectwebsock() {
	if (wsconnected){
		ws.close();
		return;
	}
	h=getvalue("wsip");
    if (h) {
        var lastcomtype = comtype;
        comtype = 1;
        var a = document.getElementById("status");
        a.innerHTML = "Web socket:Connecting...";

        function handlemessage(m) {
            msg = m.data;
            onReadCallback(msg);
            //if (debugs & 2) console.log(msg);
        }

        ws = new WebSocket('ws://' + h + ':81/', ['arduino']);
        ws.onerror = function(e) {
            comtype = lastcomtype;
            a.innerHTML = "Web socket:Failed connect ! ";
			$("wsconnect").innerHTML='Connect';
			ws.close();
			wsconnected=0;

        } // back to serial if error.
        ws.onmessage = handlemessage;
        ws.onopen = function(e) {
            console.log('Ws Connected!');
            a.innerHTML = "Web socket:Connected";
			$("wsconnect").innerHTML='Close';
            wsconnected = 1;
            nextgcode();
        };

        ws.onclose = function(e) {
            console.log('ws Disconnected!');
            a = document.getElementById("status");
            a.innerHTML = "Web socket:disconnected";
			$("wsconnect").innerHTML='Connect';
            wsconnected = 0;
        };
		idleloop();
    }
}

window.onload = function() {
	var h=window.location.host;
    a = document.activeElement;
    if ((a.tagName == "DIV") && (stotype == 1)) a.remove();
	if (h)setvalue("wsip",h);
};
