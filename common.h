#pragma once
#ifndef common_H
#define common_H

#include "motion.h"
#include <Arduino.h>


#define xprintf Serial.printf
#define zprintf Serial.printf
#define fi(x) x
#define ff(x) x



#define MASK(PIN) (1 << PIN)


extern bool canuseSerial;

/* Files for webserver */
static const char web_gcodex[] PROGMEM = 
R"EOF(function mround(e){return parseFloat(e).toFixed(3)}function mround4(e){return parseFloat(e).toFixed(4)}var packages=[],compress=[];function write(e,a){a>0&&compress.push(255&e),a>1&&compress.push(e>>8&255),a>2&&compress.push(e>>16&255),a>3&&compress.push(e>>24&255)}var cpos=0;function read(e){var a=0;if(cpos+e>=compress.length)return null;for(var t=0;t<e;t++)a+=compress[cpos+t]<<8*t;return cpos+=e,a}var holdpos=0;function hold(){holdpos=compress.length-1}var sendg1="",cntg28=0,totalgcode=0;function endcompress(){}var lx=-10,ly=-10,lz=-10,le=-10,lf=10,lF1=10,lF0=10,lS=0,lh=0,ole=0,oxmin=0,oxmax=0,oymin=0,oymax=0,ozmin=0,ozmax=0,eE=0,E=0,F0=0,F1=0,isRel=0,overtex=[],lines=[],zdeg=45,xdeg=35,sinz=.707,cosz=.707,sinx=.707,cosx=.707,filament=0,printtime=0,eScale=50,xyScale=500,zScale=10,xySize=2,zSize=1,eSize=1,fScale=2,GXVER=1;1==GXVER&&(eScale=50,xyScale=200,zScale=10,xySize=2,zSize=1,eSize=1,fScale=2);var yflip=1;function datasize(e){return(1<<8*e-1)-2}var rlx,rly,xyLimit=datasize(xySize),zLimit=datasize(zSize),eLimit=datasize(eSize),ln=0,warnoverflow=0,isF=0,repeatsame=0,xsub=0,ysub=0,zsub=0;function approx_distance(e,a){return Math.sqrt(e*e+a*a)}function writeline(e,a,t,l,i){var r,o,n;i>0&&(e|=8),(r=0!=a)&&(e|=16),(o=0!=t)&&(e|=32),(n=0!=l)&&(e|=64),write(e,1),i>0&&write(i,1),r&&write(a+xyLimit,xySize),o&&write(t+xyLimit,xySize),n&&write(l+zLimit,zSize)}function gcode_arc(e,a,t,l,i,r,o,n,c,s){var d=Math.sqrt(n*n+c*c),x=t+n,m=l+c,y=-n,h=-c,g=i-x,S=r-m,f=Math.atan2(y*S-h*g,y*g+h*S);(!s&&f<=1e-5||s&&f<-1e-6)&&(f+=2*Math.PI),s&&(f-=2*Math.PI);var z=Math.abs(f*d);if(!(z<1)){var u=Math.max(1,Math.floor(2*z/xyScale)),p=f/u;lx=t,ly=l,printtime+=Math.sqrt(z/(xyScale*xyScale))/(a*fScale);for(var M=1;M<u;M++){var v=Math.cos(M*p),b=Math.sin(M*p);y=-n*v+c*b,h=-n*b-c*v;var w=Math.floor(x+y),k=Math.floor(m+h);writeline(e,w-lx,k-ly,0,0),lx=w,ly=k,overtex.push([lx/xyScale,ly/xyScale,o/zScale,0,1,a*fScale]),oxmax=Math.max(oxmax,lx/xyScale),oxmin=Math.min(oxmin,lx/xyScale),oymax=Math.max(oymax,ly/xyScale),oymin=Math.min(oymin,ly/xyScale)}writeline(e,i-lx,r-ly,0,0),lx=i,ly=r,overtex.push([lx/xyScale,ly/xyScale,o/zScale,0,1,a*fScale])}}function gcodeline(e,a,t,l,i,r,o,n,c,s,d,x){stepx=Math.floor(o/t),stepy=Math.floor(n/t),stepz=Math.floor(c/t),steplx=o-stepx*(t-1),steply=n-stepy*(t-1),steplz=c-stepz*(t-1),0==a?(F=F0,lf=lF0):(F=F1,lf=lF1);var m=F!=lf;0==a?lF0=F0:lF1=F1;Math.abs(o),Math.abs(n),Math.abs(c);if(a<2){for(var y=1;y<=t;y++)y==t?(wx=steplx,wy=steply,wz=steplz):(wx=stepx,wy=stepy,wz=stepz),writeline(e,wx,wy,wz,1==y&&m?F:0),m=0;overtex.push([lx/xyScale,ly/xyScale,lz/zScale,0,a,lf*fScale]),printtime+=Math.sqrt((o*o+n*n)/(xyScale*xyScale))/(F*fScale)}else gcode_arc(e,lf,rlx,rly,l,i,r,s,d,2==a);rlx=lx,rly=ly}var lastG=0;function addgcode(e){if(ln++,totalgcode+=e.length,gd={},lv="",lk="",gk="",0!=(e=e.toUpperCase()).indexOf(";REP:")){for(var a=0;a<e.length&&(c=e[a],".0123456789-".indexOf(c)>=0&&(lv+=c),(" ;MGXYZEFSTRIJKP".indexOf(c)>=0||a==e.length-1)&&(lv&&(gk+=lk,gd[lk]=1*lv,lv=""),"MGXYZEFSTRIJKP".indexOf(c)>=0&&(lk=c)),";"!=c);a++);var t=0,l=0;arccw=0;var i=0;if(gk.indexOf("G")+1)switch(gd.G){case 90:isRel=0;break;case 91:isRel=1;break;case 92:i=1;break;case 2:arccw=1}if(gk.indexOf("M")+1){switch(t=1,gd.M){case 3:t|=0;break;case 5:t|=0,gd.S=0;break;case 109:t|=2;break;case 2:t|=4;break;default:return}null==(l=parseInt(gd.S))&&(l=255),t|=8;var r=gk.indexOf("X")+1,o=gk.indexOf("Y")+1,n=gk.indexOf("P")+1;if(r&&(t|=16),o&&(t|=32),n&&(t|=128),write(t,1),write(l,1),r)write(Math.round(.01*gd.X*xyScale)+xyLimit,xySize);if(o)write(Math.round(.01*gd.Y*xyScale)+xyLimit,xySize);if(n)write(Math.round(.01*gd.P),eSize);console.log(ln+" M"+gd.M+" S"+l)}else gk.indexOf("G")+1||(gk+="G",gd.G=lastG);if(gk.indexOf("G")+1){t=0;var s=gd.G;switch(lastG=s,s){case 2:t|=2,1,1;break;case 3:case 1:t|=2;break;case 0:t|=0;break;case 28:t|=4;break;case 92:break;case 192:t|=6,ole=0,E=0,eE=0,console.log("W"+E);break;default:return}var d=d||gk.indexOf("F")+1,x=(r=gk.indexOf("X")+1,o=gk.indexOf("Y")+1,gk.indexOf("Z")+1),m=gk.indexOf("I")+1,y=gk.indexOf("J")+1,h=gk.indexOf("R")+1,g=gk.indexOf("E")+1;X=Y=Z=F=S=0,dx=dy=dz=de=0,d&&!i&&(lf=1==s?F1:F0,F=Math.min(Math.round(gd.F/(60*fScale)),255),1==s?F1=F:F0=F,d=0),I=J=R=0,m&&(I=gd.I*xyScale),y&&(J=gd.J*xyScale),h&&(R=gd.R*xyScale),r&&(i?xsub=-lx+gd.X*xyScale:(X=Math.round(gd.X*xyScale-xsub),isRel&&(X+=lx),dx=X-lx,X!=lx?lx=X:r=0)),o&&(i?ysub=-ly+gd.Y*xyScale:(Y=Math.round(gd.Y*yflip*xyScale-ysub),isRel&&(Y+=ly),dy=Y-ly,Y!=ly?ly=Y:o=0)),x&&(i?zsub=-lz+gd.Z*zScale:(Z=Math.round(gd.Z*zScale-zsub),isRel&&(Z+=lz),dz=Z-lz,Z!=lz&&(lz=Z))),i&&(r||o||x||d||(xsub=0,ysub=0,zsub=0)),g&&(E=Math.round(gd.E*eScale),isRel&&(E+=le),de=E-le,le=E,filament=Math.max(le,filament));var f=t,z=Math.ceil(Math.abs(dx)/xyLimit);z=Math.max(Math.ceil(Math.abs(dy)/xyLimit),z),z=Math.max(Math.ceil(Math.abs(dz)/zLimit),z),z=Math.max(Math.ceil(Math.abs(de)/eLimit),z),y||m||h||0!=z&&28!=s?gcodeline(f,s,z,X,Y,Z,dx,dy,dz,I,J,R):(28==s?(r&&(t|=16),o&&(t|=32),x&&(t|=64),write(t,1),r&&write(Math.round(gd.X*xyScale)+xyLimit,xySize),o&&write(Math.round(gd.Y*xyScale)+xyLimit,xySize),x&&write(Math.round(gd.Z*xyScale)+xyLimit,xySize)):d&&(t=f,write(t|=8,1),write(F,1)),lh=t),oxmax=Math.max(oxmax,lx/xyScale),oxmin=Math.min(oxmin,lx/xyScale),oymax=Math.max(oymax,ly/xyScale),oymin=Math.min(oymin,ly/xyScale),ozmax=Math.max(ozmax,lz/zScale),ozmin=Math.min(ozmin,lz/zScale)}cntg28||(endcompress(),cntg28=2)}else repeatsame=parseInt(e.split(":")[1])}var decoding=0,rE=(cntg28=0,0),main_r=.5,main_g=1,main_b=.2;function randomcolor(){main_r=mround(Math.random()),main_g=mround(Math.random()),main_b=mround(Math.random());var e=Math.max(main_b,Math.max(main_r,main_g));main_r/=e,main_g/=e,main_b/=e}randomcolor();var letters="0123456789ABCDEF";function tohex(e){return e>255&&(e=255),e=Math.floor(e),letters[e>>4&15]+letters[15&e]}function getColor(e){return e<0&&(e=0),"#"+tohex(main_r*e)+tohex(main_g*e)+tohex(main_b*e)}var sp=0,pv=0;function showProgress(){}function begincompress(e,a,t){oxmax=-1e5,oxmin=1e5,oymax=-1e5,oymin=1e5,ozmax=-1e5,ozmin=1e5,xsub=0,ysub=0,zsub=0,filament=0,eE=ole=lx=ly=lz=le=0,lf=0,F1=50,F0=50,lF1=0,lF0=0,printtime=0,overtex=[],lines=[],compress=[],packages=[],isRel=0,totalgcode=0,write(71,1),write(88,1),write(GXVER,1),texts=e?e.split("\n"):getvalue("gcode").split("\n"),cntg28=2,ln=0,a&&setTimeout(a,200);for(var l=0;l<texts.length;l++)addgcode(texts[l]),!1&l&&(pv=100*l/texts.length);h=1,h|=4,write(h,1),console.log("From "+totalgcode+" to "+compress.length),console.log("Ratio "+mround(compress.length/totalgcode)),t&&t()}var AX,AY,AZ,AE,decodes="",shapes=(decoding=0,[]),shapes_ctr=0,PZ=0;function decodealine(){var e=0,a=0,t=0,l=0,i=0,r=0,o=read(1);if(null!=o){var n=0;if(1&o){switch(1,o>>1&3){case 0:t=3;break;case 1:t=109;break;case 2:return void(decoding=0)}8&o&&(e=1,i=n=read(1)),16&o&&(1,n=read(xySize)),32&o&&(1,n=read(xySize)),128&o&&(a=1,r=100*(n=read(eSize))),decodes+="M"+t,e&&(decodes+=" S"+i),a&&(decodes+=" P"+r),decodes+="\n"}else{switch(1,o>>1&3){case 0:l=0;break;case 1:l=1;break;case 2:l=28;break;case 3:l=92,rE=0}var c=0;decodes+="G"+l;var s=[];l<=1&&AZ>0&&(s=[AX,AY,AZ],PZ=AZ),8&o&&(n=read(1),1,decodes+=" F"+60*(n*fScale)),16&o&&(c=read(xySize),1,decodes+=" X"+mround(AX+=(c-xyLimit)/xyScale)),32&o&&(c=read(xySize),1,decodes+=" Y"+mround(AY+=(c-xyLimit)/xyScale)),64&o&&(28==l?(c=read(xySize),1,aZ=(c-xyLimit)/xyScale,decodes+=" Z"+mround(aZ),AZ=0):(c=read(zSize),1,decodes+=" Z"+mround(AZ+=(c-zLimit)/zScale),AZ<=0&&PZ>0&&(PZ=AZ,shapes.push(s),shapes_ctr++))),128&o&&(c=read(eSize),1,decodes+=" E"+mround(AE+=(c-eLimit)/eScale)),decodes+="\n"}0==cntg28&&(decoding=0)}else decoding=0}function decode(){for(cpos=3,cntg28=2,decoding=1,decodes="",rE=0,AX=0,AY=0,AZ=0,AE=0,shapes=[],shapes_ctr=0,PZ=0;decoding;)decodealine();return decodes}
)EOF";

static const char web_config[] PROGMEM = R"EOF( <html><body id=body style='font-family:arial'><div style='width:600px;background:#eeffaa;border-radius:10px;padding:10px;border:1px solid green;margin:auto;box-shadow: 4px 4px 2px rgba(0,0,0,0.5);'><center><h2>Karya controller configuration</h2></center><hr>Machine I.P Address <input id=wsip size=13 class=saveit value=""><table width=100%><tr><td>WIFI A.P <button onclick="scanwifi()">Scan wifi</button><td>Password<td>Machine Name<td><tr><td><select id=wifi></select> <td><input id=pwd size=10><td><input id=mid size=10><td><button onclick="setconfig()">Save Wifi</button></table><hr>Configuration &nbsp;<b id=status></b><br><button id=btreadconf>Read config</button><button style="margin-left:380px" id=btsaveconf>Save config</button><table width=100%><tr><td>Main config<td>Extra config<tr><td colspan=2><hr><tr><td><textarea style="font-size:small;overflow:scroll" wrap=off rows=13 cols=40 id=config1></textarea></td><td><textarea style="font-size:small;overflow:scroll" wrap=off rows=13 cols=40 id=config2></textarea></td></tr></table><hr><button onclick="clearjobs()">Clear all jobs !!</button><hr>Extra config, only edit existing lines, any new lines will removed<hr>Download Karyacnc <a href="https://github.com/ryannining/karyacnc/releases/download/v1.1.0/karyacnc.7z" target=dw>here</a>. After download, extract it to any new folder.Open <i>chrome://extensions</i> Settings, and enable Developer option, and click Load unpacked, select the folder of where karyacnc extracted.Extension is ready and can be open <i>chrome:://apps</i> . You can right click and create shortcut on your desptop.<br><br>Download Latest Inkscape 1.2 <a href="https://inkscape.org/release/inkscape-1.2/" target=ink>here</a>. After that you need to install KaryaCNC inkscape extension.<br>Inkscape extension (send to karyacnc) can be download <a href="inkscape.zip" target=dw>here</a>. Extract the file content to your inkscape extension folder. Usually they are in<i>Program files/Inkscape/share/extension</i> folder. You need to restart inkscape then. <font size=2><br></font></div><script>var wsconnected=0;</script><script src="websocket.js"></script><script>function mround(x){return parseFloat(x).toFixed(2);}function mround4(x){return parseFloat(x).toFixed(4);}function $(id){return document.getElementById(id);}function getvalue(el){return $(el).value;}function setvalue(el, val){$(el).value=val;}function log(text){$('log').value +=text + '\n';}function setevent(ev, bt, act){$(bt).addEventListener(ev, act, false);}function setclick(bt, act){setevent("click", bt, act);}function replaceAll(str, find, replace){return str.replace(new RegExp(find, 'g'), replace);}function urlopen(s,callback=0){if(!callback)callback=alert; var xhr=new XMLHttpRequest(); xhr.open( "GET","/"+s,true); xhr.onload=function(e){callback(xhr.response);}; xhr.send();}function realupload(data, fname, callback){var xhr=new XMLHttpRequest();form=new FormData();form.append("file1", data, fname);xhr.open("POST","http://" + $("wsip").value + "/upload",true);var progressBar=$('progress1');xhr.addEventListener('error', function(e){});xhr.onload=function(e){if (callback) callback();};xhr.upload.onprogress=function(e){if (e.lengthComputable){}};xhr.send(form);}function dbmtohuman(dbm){dbm=parseFloat(dbm);if (dbm<=-80)return "Not good";if (dbm<=-70)return "Okay";if (dbm<=-67)return "Very good";return "Amazing";}function scanwifi(){urlopen("scanwifi",function(data){var wifi=eval(data);var op="";for (var i=0;i<wifi.length;i++){op+="<option value='"+wifi[i][0]+"'>"+wifi[i][0]+" ("+dbmtohuman(wifi[i][1])+") "+wifi[i][2]+"</option>";}$("wifi").innerHTML=op;getconfig();});}function getconfig(){urlopen("getconfig",function(data){var wifi=eval(data);if ($('wifi').innerHTML==''){$('wifi').innerHTML="<option selected value='"+wifi[0]+"'>"+wifi[0]+"</option>";}else $("wifi").value=wifi[0];$("pwd").value=wifi[1];$("mid").value=wifi[2];});}function setconfig(){urlopen("setconfig?name="+(mid.value)+"&ap="+$('wifi').value+"&pw="+$('pwd').value);}function clearjobs(){if (confirm("Delete all jobs ?"))urlopen("clearjobs");}setclick("btreadconf", function(){urlopen("config.ini",function(e){$("config1").value=e;}); urlopen("custom.ini",function(e){if (e!="404: Not Found ?")$("config2").value=e;});});function upload3(){alert("Upload config done");}function upload2(){realupload(new Blob([$("config2").value],{type: "text/plain"}), "/custom.ini", upload3);};setclick("btsaveconf", function(){realupload(new Blob([$("config1").value],{type: "text/plain"}), "/config.ini", upload2);});</script></body></html>
)EOF";
static const char web_websocket[] PROGMEM = R"EOF(
 var wsconnected=0,lastw="",oktosend=1,connectionId=null,eline=0,okwait=0,running=0,px=0,py=0,pz=0,pe=0,etime=new Date,checktemp=1,comtype=0,egcodes=[],debugs=2;function sendgcode(e){1&debugs&&console.log(e);try{0==comtype&&writeSerial(e+"\n"),1==comtype&&ws.send(e+"\n")}catch(e){}}function nextgcode(){if((1!=comtype||wsconnected)&&!(0<okwait)&&running){for(;eline<egcodes.length;){var e=egcodes[eline];if(eline++,";PAUSE"==e&&pause(),e&&";"!=e[0]&&"("!=e[0])return okwait=1,void sendgcode(e.split(";")[0])}sendgcode("G4"),sendgcode("M114"),stopit()}}function stopit(){var e=etime.getTime();etime=new Date,console.log("Stop "+etime),e=etime.getTime()-e,mss="Real time:"+mround(e/6e4),console.log(mss),document.getElementById("btexecute").innerHTML="Print",document.getElementById("btpause").innerHTML="Pause",okwait=running=0,egcodes=[]}function execute(e){etime=new Date,console.log("Start "+etime),document.getElementById("btpause").innerHTML="PAUSE",egcodes=e.split("\n"),eline=0,running=egcodes.length,okwait=0,nextgcode()}function idleloop(){wsconnected&&checktemp&&(checktemp=0),setTimeout(idleloop,3e3)}var ss="",eeprom={},ineeprom=0,eppos=0,resp1="",onReadCallback=function(e){resp1+=e;for(var n="",o=0;o<e.length;o++)if("\n"==e[o]?(0<ss.indexOf("Z:")&&(px=parseFloat(ss.substr(ss.indexOf("X:")+2)),py=parseFloat(ss.substr(ss.indexOf("Y:")+2)),pz=parseFloat(ss.substr(ss.indexOf("Z:")+2)),pe=parseFloat(ss.substr(ss.indexOf("E:")+2))),2&debugs&&console.log(ss),ss=""):ss+=e[o],"\n"==e[o]||" "==e[o]||"*"==e[o]){if(0<ineeprom){if(20==ineeprom)eppos=lastw;else if(19==ineeprom)eeprom[eppos]=lastw;else{if(n+=lastw+" ","\n"==e[o])document.getElementById("eepromid").innerHTML+='<option value="'+eeprom[eppos]+":"+eppos+'">'+n+"</option>",ineeprom=1}ineeprom--}0<=lastw.toUpperCase().indexOf("EPR:")&&(ineeprom=20,n=""),0<=lastw.toUpperCase().indexOf("T:")&&(document.getElementById("info3d").innerHTML=lastw,checktemp=1),isok=2==lastw.length&&"O"==lastw[0].toUpperCase(),(isok||0<=lastw.toUpperCase().indexOf("OK")||0<=lastw.toUpperCase().indexOf("WAIT"))&&(okwait=0,nextgcode()),lastw=""}else lastw+=e[o]};function connectwebsock(){if(wsconnected)ws.close();else if(h=getvalue("wsip"),h){var n=comtype;comtype=1;var o=document.getElementById("status");o.innerHTML="Web socket:Connecting...",ws=new WebSocket("ws://"+h+":81/",["arduino"]),ws.onerror=function(e){comtype=n,o.innerHTML="Web socket:Failed connect ! ",$("wsconnect").innerHTML="Connect",ws.close(),wsconnected=0},ws.onmessage=function(e){msg=e.data,onReadCallback(msg)},ws.onopen=function(e){console.log("Ws Connected!"),o.innerHTML="Web socket:Connected",$("wsconnect").innerHTML="Close",wsconnected=1,nextgcode()},ws.onclose=function(e){console.log("ws Disconnected!"),(o=document.getElementById("status")).innerHTML="Web socket:disconnected",$("wsconnect").innerHTML="Connect",wsconnected=0},idleloop()}}window.onload=function(){var e=window.location.host;a=document.activeElement,"DIV"==a.tagName&&1==stotype&&a.remove(),e&&setvalue("wsip",e)};
)EOF";


static const char web_index[] PROGMEM = 
R"EOF(<meta content="width=device-width,initial-scale=1"name=viewport><title>Machine JOBS</title><style>.smallwindow{font-size:9pt;font-family:arial;text-align:right;background:#0ff;display:none;position:absolute;border:1px solid #00f;box-shadow:4px 4px 2px rgba(0,0,0,.5);padding:5px;z-index:1000}#editorengcode,#editorgcode{position:absolute;top:0;right:0;bottom:0;left:0}.j1{border:1px solid;border-radius:5px;width:40%;height:auto;padding:10px;margin:5px;display:inline-block}.j3{border:1px solid;border-radius:5px;width:40%;height:auto;padding:10px;margin:5px;display:inline-block;color:red}.j2{border:1px solid;border-radius:5px;width:40%;height:auto;padding:10px;margin:5px;display:inline-block;background:#acf}.jog{padding:5px;min-width:60}body,html{overflow-y:visible}</style><body style=font-family:arial;font-size:12pt;margin:auto><div class=j1 style=width:80%><b>JOG MACHINE</b>    |   <a href=cnc.html target=new>New Job</a><hr><table width=100%><tr align=center><td><button class=jog onclick=speed1()>1x</button><td><button class=jog onclick=jogup()>Up</button><td><button class=jog onclick=zup()>Z+1</button> <button class=jog onclick=z15()>Z 15</button><tr align=center><td><button class=jog onclick=jogleft()>Left</button><td><select id=step><option value=.25>.5<option value=1>1<option value=5 selected>5<option value=25>25<option value=50>50<option value=150>150</select> mm<td><button class=jog onclick=jogright()>Right</button> <button class=jog onclick=probe()>Probe</button><tr align=center><td><button class=jog onclick=speed2()>1.5x</button><td><button class=jog onclick=jogdown()>Down</button><td><button class=jog onclick=zdown()>Z-1</button> <button class=jog onclick=z0()>Z 0</button></table><hr><hr><button class=jog onclick=jogpause()>Pause Job</button> <button class=jog onclick=jogstop()>Stop Job</button> <button class=jog onclick=joghome()>HOME</button> <button class=jog onclick=jogzero()>ZERO</button><hr><button class=jog onclick=nobedlevel()>No Mesh</button> <button class=jog onclick=bedlevel(200,1200,1200)>Mesh A.Lv</button> <button class=jog onclick=loadbedlevel()>Load mesh</button><hr><button class=jog onclick=alert2a()>Reload jobs</button> <button class=jog onclick=spindle(255)>Spindle On</button> <button class=jog onclick=spindle(0)>Spindle Off</button> <button class=jog onclick=spindle(125)>Test Laser</button><hr></div><br>Show preview <input id=preview onclick=joblist(lastres) type=checkbox>  Filter <input id=dofilter onclick=joblist(lastres) type=checkbox> <select id=filter onchange=joblist(lastres)><option value=mdf>mdf<option value=cnc>cnc<option value=acp>acp<option value=job>job<option value=mika>mika<option value=test>test<option value=jig>jig</option><select><hr><div id=jobs></div><script>var wsip="localhost",jobs;function $(e){return document.getElementById(e)}function getvalue(e){return"gcode"==e?editorgcode.getValue():"engcode"==e?editorengcode.getValue():$(e).value}function mround(e){return parseFloat(e).toFixed(1)}var rjob=-1;function urlopen(e,o){var n=new XMLHttpRequest;n.open("GET","http://"+wsip+"/"+e,!0),n.onload=function(e){o&&o(n.response)},n.send()}function alert2(e){var o="";if(0<=rjob){o=jobs[rjob][0],j1s=document.getElementsByClassName("j1");for(var n=0;n<j1s.length;n++)j1s[n].classList.remove("j2");$("job"+rjob).classList.add("j3"),$("job"+rjob).classList.add("j2")}var t=new Date;alert("("+o+":"+t+") "+e),rjob=-1}function nobedlevel(){urlopen("gcode?t=G29",alert2)}function bedlevel(e,o,n){urlopen("gcode?t=G30 S"+e+" X"+o+" Y"+n,alert2)}function loadbedlevel(){urlopen("gcode?t=G31",alert2)}function setxbc(){urlopen("gcode?t=M206 P80 S"+getvalue("xbacklash"),function(e){urlopen("gcode?t=M206 P84 S"+getvalue("ybacklash"),alert2)})}function setybc(){}function speed1(){urlopen("speed?t=100")}function speed2(){urlopen("speed?t=150")}function spindle(e){urlopen("tool?t="+e)}function zup(){zz=1*getvalue("step"),1<zz&&(zz=1),urlopen("jogmove?z="+zz+"&y=0&x=0")}function zdown(){zz=1*getvalue("step"),1<zz&&(zz=1),urlopen("jogmove?z=-"+zz+"&y=0&x=0")}function z15(){urlopen("gcode?t=G0 Z15 F800")}function z0(){urlopen("gcode?t=G0 Z0 F800")}function probe(){urlopen("probe",alert2)}function jogleft(){urlopen("jogmove?z=0&y=0&x=-"+getvalue("step"))}function jogright(){urlopen("jogmove?z=0&y=0&x="+getvalue("step"))}function jogup(){urlopen("jogmove?z=0&x=0&y=-"+getvalue("step"))}function jogdown(){urlopen("jogmove?z=0&x=0&y="+getvalue("step"))}function jogpause(){urlopen("pauseprint",alert2)}function jogstop(){confirm("Stop job ?")&&urlopen("stopprint",alert2)}function joghome(){urlopen("home")}function jogzero(){confirm("Set position as 0,0,0 ?")&&urlopen("gcode?t=G92",alert2)}function runjob(e){rjob=e,confirm("Start job "+jobs[e][0]+" ?")&&urlopen("startjob?jobname="+jobs[e][0],alert2)}function previewjob(e){rjob=e,confirm("Preview job "+jobs[e][0]+" ?")&&urlopen("previewjob?jobname="+jobs[e][0],alert2)}function alert2a(e){urlopen("getjobs",joblist)}function deljob(e){var o=jobs[e][0],n=prompt("Change name to rename",o);n&&urlopen(n!=o?"renamejob?jobname="+o+"&newname="+n:"removejob?jobname="+o,alert2a)}var lastres="";function joblist(res){var j="";if("ERR"==res)j="Still running job !";else{var dofilt=$("dofilter").checked,filt=$("filter").value.toUpperCase();lastres=res,jobs=eval(res).sort();var ipre=$("preview").checked;for(i in jobs){if(dofilt){var ju=jobs[i][0].toUpperCase();if(ju.indexOf(filt)<0)continue}var sz=" "+mround(jobs[i][1]/1e3)+"Kb";jn=jobs[i][0].split(".g")[0];var img="<a href='http://"+wsip+"/cnc?view="+jobs[i][0]+"' target=preview ><button>View</button></a>";j+="<div class=j1 id='job"+i+"'><b>"+jn+"</b><hr>"+img+"<button onclick='runjob("+i+")'>Start</button><button onclick='previewjob("+i+")'>Reveal</button><button onclick='deljob("+i+")'>Remove</button></div>"}}$("jobs").innerHTML=j}window.onload=function(){var e=window.location.host;wsip="192.168.0.104",e&&(wsip=e),urlopen("getjobs",joblist)}</script>)EOF";


static const char web_cnc[] PROGMEM = 
R"EOF(<html>

<body id=body style='font-family:arial'>
<div style='width:600px;background:#eeffaa;border-radius:10px;padding:10px;border:1px solid green;margin:auto;box-shadow: 4px 4px 2px rgba(0,0,0,0.5);'>
<style>
#xdecode { 
        position: absolute;
        top: 0;
        right: 0;
        bottom: 0;
        left: 0;
    }
</style>
<table width=100%><tr valign=top><td>
<div>LOAD GCODE AND PREVIEW</div><hr>
<input type="file" id="inputfile"/><br>
View angle<input id=angle value="0,90" size=5><button onclick="init2d(0);rendergcode();">View</button>
<div>Preview:</div>
<canvas id="prevCanvas" width="400" height="300" style="border:1px solid #d3d3d3;"></canvas>
<table width=200><tr><td>
<td align=right>
</table>
<td>
<div>UPLOAD TO MACHINE</div><hr>
Machine IP <input id=wsip size=14 class=saveit value="">
<div id=info>Info:</div>
Job name <input id=jobname size=10 value=gcode><br>
<button id=btclick onclick="upload()">Upload</button> <progress id=progress1 min="0" max="100" value="0" style="width:100px">0% complete</progress><br>
<button id=btclick2 onclick="download()">Download</button>

</table>
<hr>You can also just copy paste the Gcode Text here. Flip Y <input type=checkbox checked id=isflipy>
 <div style="rfont-size: 10;width:100%;height:300" id="gcode">
<textarea style="width:100%;height:100%" id="gcodet"></textarea>
</div>

Convert to SVG, from line:<input id="fromline" size="5" value="0"><br>

    <button onclick="convertToSVG()">Convert to SVG</button>
    <button onclick="downloadSVG()">Download SVG</button>

    <div id="svgContainer"></div>
</div>
<script>
var wsconnected=0;
</script>
<script src="websocket.js"></script>
<script src="--ace.js" type="text/javascript" charset="utf-8"></script>
<script src="--gcodex.js" type="text/javascript" charset="utf-8"></script>
<script>

function mround(e){return parseFloat(e).toFixed(3)}function mround4(e){return parseFloat(e).toFixed(4)}var packages=[],compress=[];function write(e,a){a>0&&compress.push(255&e),a>1&&compress.push(e>>8&255),a>2&&compress.push(e>>16&255),a>3&&compress.push(e>>24&255)}var cpos=0;function read(e){var a=0;if(cpos+e>=compress.length)return null;for(var t=0;t<e;t++)a+=compress[cpos+t]<<8*t;return cpos+=e,a}var holdpos=0;function hold(){holdpos=compress.length-1}var sendg1="",cntg28=0,totalgcode=0;function endcompress(){}var lx=-10,ly=-10,lz=-10,le=-10,lf=10,lF1=10,lF0=10,lS=0,lh=0,ole=0,oxmin=0,oxmax=0,oymin=0,oymax=0,ozmin=0,ozmax=0,eE=0,E=0,F0=0,F1=0,isRel=0,overtex=[],lines=[],zdeg=45,xdeg=35,sinz=.707,cosz=.707,sinx=.707,cosx=.707,filament=0,printtime=0,eScale=50,xyScale=500,zScale=10,xySize=2,zSize=1,eSize=1,fScale=2,GXVER=1;1==GXVER&&(eScale=50,xyScale=200,zScale=10,xySize=2,zSize=1,eSize=1,fScale=2);var yflip=1;function datasize(e){return(1<<8*e-1)-2}var rlx,rly,xyLimit=datasize(xySize),zLimit=datasize(zSize),eLimit=datasize(eSize),ln=0,warnoverflow=0,isF=0,repeatsame=0,xsub=0,ysub=0,zsub=0;function approx_distance(e,a){return Math.sqrt(e*e+a*a)}function writeline(e,a,t,l,i){var r,o,n;i>0&&(e|=8),(r=0!=a)&&(e|=16),(o=0!=t)&&(e|=32),(n=0!=l)&&(e|=64),write(e,1),i>0&&write(i,1),r&&write(a+xyLimit,xySize),o&&write(t+xyLimit,xySize),n&&write(l+zLimit,zSize)}function gcode_arc(e,a,t,l,i,r,o,n,c,s){var d=Math.sqrt(n*n+c*c),x=t+n,m=l+c,y=-n,h=-c,g=i-x,S=r-m,f=Math.atan2(y*S-h*g,y*g+h*S);(!s&&f<=1e-5||s&&f<-1e-6)&&(f+=2*Math.PI),s&&(f-=2*Math.PI);var z=Math.abs(f*d);if(!(z<1)){var u=Math.max(1,Math.floor(2*z/xyScale)),p=f/u;lx=t,ly=l,printtime+=Math.sqrt(z/(xyScale*xyScale))/(a*fScale);for(var M=1;M<u;M++){var v=Math.cos(M*p),b=Math.sin(M*p);y=-n*v+c*b,h=-n*b-c*v;var w=Math.floor(x+y),k=Math.floor(m+h);writeline(e,w-lx,k-ly,0,0),lx=w,ly=k,overtex.push([lx/xyScale,ly/xyScale,o/zScale,0,1,a*fScale]),oxmax=Math.max(oxmax,lx/xyScale),oxmin=Math.min(oxmin,lx/xyScale),oymax=Math.max(oymax,ly/xyScale),oymin=Math.min(oymin,ly/xyScale)}writeline(e,i-lx,r-ly,0,0),lx=i,ly=r,overtex.push([lx/xyScale,ly/xyScale,o/zScale,0,1,a*fScale])}}function gcodeline(e,a,t,l,i,r,o,n,c,s,d,x){stepx=Math.floor(o/t),stepy=Math.floor(n/t),stepz=Math.floor(c/t),steplx=o-stepx*(t-1),steply=n-stepy*(t-1),steplz=c-stepz*(t-1),0==a?(F=F0,lf=lF0):(F=F1,lf=lF1);var m=F!=lf;0==a?lF0=F0:lF1=F1;Math.abs(o),Math.abs(n),Math.abs(c);if(a<2){for(var y=1;y<=t;y++)y==t?(wx=steplx,wy=steply,wz=steplz):(wx=stepx,wy=stepy,wz=stepz),writeline(e,wx,wy,wz,1==y&&m?F:0),m=0;overtex.push([lx/xyScale,ly/xyScale,lz/zScale,0,a,lf*fScale]),printtime+=Math.sqrt((o*o+n*n)/(xyScale*xyScale))/(F*fScale)}else gcode_arc(e,lf,rlx,rly,l,i,r,s,d,2==a);rlx=lx,rly=ly}var lastG=0;function addgcode(e){if(ln++,totalgcode+=e.length,gd={},lv="",lk="",gk="",0!=(e=e.toUpperCase()).indexOf(";REP:")){for(var a=0;a<e.length&&(c=e[a],".0123456789-".indexOf(c)>=0&&(lv+=c),(" ;MGXYZEFSTRIJKP".indexOf(c)>=0||a==e.length-1)&&(lv&&(gk+=lk,gd[lk]=1*lv,lv=""),"MGXYZEFSTRIJKP".indexOf(c)>=0&&(lk=c)),";"!=c);a++);var t=0,l=0;arccw=0;var i=0;if(gk.indexOf("G")+1)switch(gd.G){case 90:isRel=0;break;case 91:isRel=1;break;case 92:i=1;break;case 2:arccw=1}if(gk.indexOf("M")+1){switch(t=1,gd.M){case 3:t|=0;break;case 5:t|=0,gd.S=0;break;case 109:t|=2;break;case 2:t|=4;break;default:return}null==(l=parseInt(gd.S))&&(l=255),t|=8;var r=gk.indexOf("X")+1,o=gk.indexOf("Y")+1,n=gk.indexOf("P")+1;if(r&&(t|=16),o&&(t|=32),n&&(t|=128),write(t,1),write(l,1),r)write(Math.round(.01*gd.X*xyScale)+xyLimit,xySize);if(o)write(Math.round(.01*gd.Y*xyScale)+xyLimit,xySize);if(n)write(Math.round(.01*gd.P),eSize);console.log(ln+" M"+gd.M+" S"+l)}else gk.indexOf("G")+1||(gk+="G",gd.G=lastG);if(gk.indexOf("G")+1){t=0;var s=gd.G;switch(lastG=s,s){case 2:t|=2,1,1;break;case 3:case 1:t|=2;break;case 0:t|=0;break;case 28:t|=4;break;case 92:break;case 192:t|=6,ole=0,E=0,eE=0,console.log("W"+E);break;default:return}var d=d||gk.indexOf("F")+1,x=(r=gk.indexOf("X")+1,o=gk.indexOf("Y")+1,gk.indexOf("Z")+1),m=gk.indexOf("I")+1,y=gk.indexOf("J")+1,h=gk.indexOf("R")+1,g=gk.indexOf("E")+1;X=Y=Z=F=S=0,dx=dy=dz=de=0,d&&!i&&(lf=1==s?F1:F0,F=Math.min(Math.round(gd.F/(60*fScale)),255),1==s?F1=F:F0=F,d=0),I=J=R=0,m&&(I=gd.I*xyScale),y&&(J=gd.J*xyScale),h&&(R=gd.R*xyScale),r&&(i?xsub=-lx+gd.X*xyScale:(X=Math.round(gd.X*xyScale-xsub),isRel&&(X+=lx),dx=X-lx,X!=lx?lx=X:r=0)),o&&(i?ysub=-ly+gd.Y*xyScale:(Y=Math.round(gd.Y*yflip*xyScale-ysub),isRel&&(Y+=ly),dy=Y-ly,Y!=ly?ly=Y:o=0)),x&&(i?zsub=-lz+gd.Z*zScale:(Z=Math.round(gd.Z*zScale-zsub),isRel&&(Z+=lz),dz=Z-lz,Z!=lz&&(lz=Z))),i&&(r||o||x||d||(xsub=0,ysub=0,zsub=0)),g&&(E=Math.round(gd.E*eScale),isRel&&(E+=le),de=E-le,le=E,filament=Math.max(le,filament));var f=t,z=Math.ceil(Math.abs(dx)/xyLimit);z=Math.max(Math.ceil(Math.abs(dy)/xyLimit),z),z=Math.max(Math.ceil(Math.abs(dz)/zLimit),z),z=Math.max(Math.ceil(Math.abs(de)/eLimit),z),y||m||h||0!=z&&28!=s?gcodeline(f,s,z,X,Y,Z,dx,dy,dz,I,J,R):(28==s?(r&&(t|=16),o&&(t|=32),x&&(t|=64),write(t,1),r&&write(Math.round(gd.X*xyScale)+xyLimit,xySize),o&&write(Math.round(gd.Y*xyScale)+xyLimit,xySize),x&&write(Math.round(gd.Z*xyScale)+xyLimit,xySize)):d&&(t=f,write(t|=8,1),write(F,1)),lh=t),oxmax=Math.max(oxmax,lx/xyScale),oxmin=Math.min(oxmin,lx/xyScale),oymax=Math.max(oymax,ly/xyScale),oymin=Math.min(oymin,ly/xyScale),ozmax=Math.max(ozmax,lz/zScale),ozmin=Math.min(ozmin,lz/zScale)}cntg28||(endcompress(),cntg28=2)}else repeatsame=parseInt(e.split(":")[1])}var decoding=0,rE=(cntg28=0,0),main_r=.5,main_g=1,main_b=.2;function randomcolor(){main_r=mround(Math.random()),main_g=mround(Math.random()),main_b=mround(Math.random());var e=Math.max(main_b,Math.max(main_r,main_g));main_r/=e,main_g/=e,main_b/=e}randomcolor();var letters="0123456789ABCDEF";function tohex(e){return e>255&&(e=255),e=Math.floor(e),letters[e>>4&15]+letters[15&e]}function getColor(e){return e<0&&(e=0),"#"+tohex(main_r*e)+tohex(main_g*e)+tohex(main_b*e)}var sp=0,pv=0;function showProgress(){}function begincompress(e,a,t){oxmax=-1e5,oxmin=1e5,oymax=-1e5,oymin=1e5,ozmax=-1e5,ozmin=1e5,xsub=0,ysub=0,zsub=0,filament=0,eE=ole=lx=ly=lz=le=0,lf=0,F1=50,F0=50,lF1=0,lF0=0,printtime=0,overtex=[],lines=[],compress=[],packages=[],isRel=0,totalgcode=0,write(71,1),write(88,1),write(GXVER,1),texts=e?e.split("\n"):getvalue("gcode").split("\n"),cntg28=2,ln=0,a&&setTimeout(a,200);for(var l=0;l<texts.length;l++)addgcode(texts[l]),!1&l&&(pv=100*l/texts.length);h=1,h|=4,write(h,1),console.log("From "+totalgcode+" to "+compress.length),console.log("Ratio "+mround(compress.length/totalgcode)),t&&t()}var AX,AY,AZ,AE,decodes="",shapes=(decoding=0,[]),shapes_ctr=0,PZ=0;function decodealine(){var e=0,a=0,t=0,l=0,i=0,r=0,o=read(1);if(null!=o){var n=0;if(1&o){switch(1,o>>1&3){case 0:t=3;break;case 1:t=109;break;case 2:return void(decoding=0)}8&o&&(e=1,i=n=read(1)),16&o&&(1,n=read(xySize)),32&o&&(1,n=read(xySize)),128&o&&(a=1,r=100*(n=read(eSize))),decodes+="M"+t,e&&(decodes+=" S"+i),a&&(decodes+=" P"+r),decodes+="\n"}else{switch(1,o>>1&3){case 0:l=0;break;case 1:l=1;break;case 2:l=28;break;case 3:l=92,rE=0}var c=0;decodes+="G"+l;var s=[];l<=1&&AZ>0&&(s=[AX,AY,AZ],PZ=AZ),8&o&&(n=read(1),1,decodes+=" F"+60*(n*fScale)),16&o&&(c=read(xySize),1,decodes+=" X"+mround(AX+=(c-xyLimit)/xyScale)),32&o&&(c=read(xySize),1,decodes+=" Y"+mround(AY+=(c-xyLimit)/xyScale)),64&o&&(28==l?(c=read(xySize),1,aZ=(c-xyLimit)/xyScale,decodes+=" Z"+mround(aZ),AZ=0):(c=read(zSize),1,decodes+=" Z"+mround(AZ+=(c-zLimit)/zScale),AZ<=0&&PZ>0&&(PZ=AZ,shapes.push(s),shapes_ctr++))),128&o&&(c=read(eSize),1,decodes+=" E"+mround(AE+=(c-eLimit)/eScale)),decodes+="\n"}0==cntg28&&(decoding=0)}else decoding=0}function decode(){for(cpos=3,cntg28=2,decoding=1,decodes="",rE=0,AX=0,AY=0,AZ=0,AE=0,shapes=[],shapes_ctr=0,PZ=0;decoding;)decodealine();return decodes}

var c = document.getElementById("prevCanvas");
var prevCtx = c.getContext("2d");


function $(id) {
    return document.getElementById(id);
}
function getvalue(el) {
    return $(el).value;
}

function setvalue(el, val) {
    $(el).value = val;
}
function log(text) {
    $('log').value += text + '\n';
}


function setevent(ev, bt, act) {
    $(bt).addEventListener(ev, act, false);
}

function setclick(bt, act) {
    setevent("click", bt, act);
}

function replaceAll(str, find, replace) {
    return str.replace(new RegExp(find, 'g'), replace);
}
var packages=[];
var compress=[];
function write(w,s){
	if (s>0)compress.push(w & 255);
	if (s>1)compress.push((w >> 8) & 255);
	if (s>2)compress.push((w >> 16) & 255);	
}
var holdpos=0;
function hold(){
	holdpos=compress.length-1;
}

var sendg1="";
function sendg(){
	var s=prompt("GCODE Command",sendg1);
	if (s) {
		sendg1=s;
		sendgcode(s);
	}
}
function executedecode(){
    var bt = document.getElementById('btexecute');
    if (bt.innerHTML == "Print") {
        execute(editor.getValue());
        bt.innerHTML = "Stop";
    } else {
        stopit();
        sendgcode("M2");
    }
}
function pause() {
    var bt = document.getElementById('btpause');
    if (bt.innerHTML == "PAUSE") {
        running = 0;
        sendgcode("M3 S200 P10");
        bt.innerHTML = "RESUME";
    } else {
        running = 1;
        sendgcode("M3 S200");
        nextgcode();
        bt.innerHTML = "PAUSE";
    }
}
function hardstop() {
    sendgcode("M2");
    sendgcode("G0 Z2 F5000");
    sendgcode("G0 X0 Y0");
    px=0;
	py=0;
	pz=2;
    stopit();
}

function init2d(v){
  //return;	
  if (!v){
	  v=(getvalue("angle")).split(",");
	  zdeg=v[0]*1;
	  xdeg=v[1]*1;
  } else zdeg+=v;	
  sinz=Math.sin(zdeg*0.0174533);
  cosz=Math.cos(zdeg*0.0174533);
  sinx=Math.sin(xdeg*0.0174533);
  cosx=Math.cos(xdeg*0.0174533);
}
init2d(0);

function transform(v3d){
	var x2=v3d[0]*cosz-v3d[1]*sinz;
	var y1=v3d[0]*sinz+v3d[1]*cosz;
	
	var y2=y1*cosx-v3d[2]*sinx;
	var z2=y1*sinx+v3d[2]*cosx;
	
	return [x2,y2,z2];
	
}


function rendergcode(canvasid){
	if (overtex.lentgh==0)return;
	xmax=-100000;
	xmin=100000;
	ymax=-100000;
	ymin=100000;
	zmax=-100000;
	zmin=100000;
	

	var vertex=[];
	lines=[];
	var vc=[];
	var lv=0;
	// 3d to 2d transformation + lighting
	for (var i=0;i<overtex.length;i++){
		var ov=overtex[i];
		
        //ov[3]=-ov[3];
		var tr=transform(ov);
		
		var tx=tr[0];
		var ty=tr[2];
		// save bounding box


		// save vertex
		vc=[tx,ty,-tr[1]];
		vertex.push(vc);
		if (ov[4]==1 && lv){
		    // cheap fast lighting
		    dx=lv[0]-vc[0];
		    dz=lv[1]-vc[1];
		    dy=lv[2]-vc[2];
		    
		    l=-ov[3];//Math.abs(150*dx/Math.sqrt(dx*dx+dy*dy));
		    // if there is extruder value then save the line
		    p=vertex.length-1;
		    zz=0;//-Math.min(vertex[p][2],vertex[p-1][2]);
		    lines.push([p-1,p,l,zz,ov[5]]);
		    xmax=Math.max(xmax,tx);		
		    xmin=Math.min(xmin,tx);		
		    ymax=Math.max(ymax,ty);		
		    ymin=Math.min(ymin,ty);		
		    zmax=Math.max(zmax,tr[1]);		
		    zmin=Math.min(zmin,tr[1]);
		}
		lv=vc;
	}
	prevCtx.clearRect(0, 0, prevCtx.canvas.width, prevCtx.canvas.height);
    ///*/ nice background
	var grd=prevCtx.createLinearGradient(0,0,0,200);
	grd.addColorStop(0,"#88bbFF");
	grd.addColorStop(0.35,"white");
	grd.addColorStop(0.55,"#eeaa33");
	grd.addColorStop(1,"#bb7711");

	prevCtx.fillStyle="white";
	prevCtx.fillRect(0,0,prevCtx.canvas.width,prevCtx.canvas.height);
	//*/
  
	//calculate scale
	var zs=200.0/(zmax-zmin);
	var xs=prevCtx.canvas.width/(xmax-xmin);
	var ys=prevCtx.canvas.height/(ymax-ymin);
	
	var sc = Math.min(xs,ys);
	
	var xa=-xmin*sc+(prevCtx.canvas.width/2-0.5*(xmax-xmin)*sc);
	var za=-zmin*sc;
	var ya=-ymin*sc+(prevCtx.canvas.height/2-0.5*(ymax-ymin)*sc);
	var rp=1;Math.floor(2000/(ozmax-ozmin))+1;
	
	// actual rendering
	var front=[];
	for (var i=0;i<lines.length;i++){
		var v1=vertex[lines[i][0]];
		var v2=vertex[lines[i][1]];
		// light color
		//ll=getColor(lines[i][2]+v1[2]*zs+za);
		ll="#40404080";
		if (lines[i][4]>150)ll="#c0404080";
		else if (lines[i][4]>20)ll="#4040c080";
		//else (lines[i][4]>150)ll="blue";
		
		// transform to canvas size
		var x1=v1[0]*sc+xa+3;
		var x2=v2[0]*sc+xa+3;
		var y1=v1[1]*sc+ya+3;
		var y2=v2[1]*sc+ya+3;
		for (var j=0;j<rp;j++){
			prevCtx.beginPath();
			
			prevCtx.strokeStyle=ll;
			prevCtx.moveTo(x1, y1-j);
			prevCtx.lineTo(x2, y2-j);
			prevCtx.stroke();
		}
		
	}
    prevCtx.font = "11px Arial";
    prevCtx.fillStyle = "white";
    sx = mround((oxmax - oxmin) / 10);
    sy = mround((oymax - oymin) / 10);
    sz = mround((ozmax - ozmin) / 10);
	gram=filament/(333*eScale);
mtr=mround(filament/(eScale*1000));
    prevCtx.fillText("X:" + sx + "  Y:" + sy +"  Z:" + sz + "cm",3,11);
	prevCtx.fillText("Time:"+mround(printtime/60)+"m", 3, 23);
    prevCtx.fillStyle = "black";
    prevCtx.fillText("X:" + sx + "  Y:" + sy +"  Z:" + sz + "cm",2,10);
	prevCtx.fillText("Time:"+mround(printtime/60)+"m", 2, 22);
	
}

var sp=0;
var pv=0;
var cs="";
function showProgress(){
}

var t0=performance.now();
function viewgcode(){
  if(typeof editorgcode == 'undefined') $("gcodet").innerHTML=decode();else editorgcode.setValue(decode(), -1);
}
function aftercompress(){
	var t1=performance.now();
	c1=mround((t1-t0)/1000);
	lines.sort(function (b,a){return a[3]-b[3];})
	rendergcode();
	t0=performance.now();
	c2=mround((t0-t1)/1000);
	
	$("info").innerHTML=mround(totalgcode/1000)+" > "+mround(compress.length/1000)+" Kbyte<br>Ratio "+mround(totalgcode/compress.length)+"<br>Compress "+c1+"seconds"+"<br>Render "+c2+"seconds";
	//progressBar.value=0;
	var s="";
	for (var i=0;i<compress.length;i++){s+=String.fromCharCode(compress[i]);}
	cs=JSON.stringify(s);
	viewgcode();
}
function xurlopen(s,cb){
  var xhr = new XMLHttpRequest();
  xhr.open( "GET","/"+s,true);
  xhr.onload = cb;
  xhr.send();
}
function urlopen(s){
  xurlopen(s, function(e) {alert(xhr.response);});
}
function startprint(){
  urlopen("startprint");
}
function stopprint(){
  urlopen("stopprint");
}
function pauseprint(){
  urlopen("pauseprint");
}
function resumeprint(){
  urlopen("resumeprint");
}
function heating(){
  urlopen("heating");
}
function cooling(){
  urlopen("cooling");
}
function homing(){
  urlopen("home");
}

var mbody=document.getElementById("body");
var gcode="";
function myFunction(v) {
        yflip=$("isflipy").checked?-1:1;
	if (document.activeElement==$("decode"))return;
	gcode=v.clipboardData.getData('text/plain');
	if (gcode){
		t0=performance.now();
		begincompress(gcode,showProgress,aftercompress);
		//$("gcode").innerHTML=s;
	}
}

function decode2(){
	viewgcode();
}
mbody.addEventListener("paste", myFunction);
mbody.ondrop = function(e) {
    this.className = '';
    e.preventDefault();

    var file = e.dataTransfer.files[0];
    var gcodex=file.name.indexOf(".gcodex")>0;
    var reader = new FileReader();

    reader.onload = function(event) {
	  if (gcodex){
	    compress=new Uint8Array(event.target.result);
	    aftercompress();
	  } else begincompress(event.target.result,showProgress,aftercompress);
    };
    if (gcodex) {
      reader.readAsArrayBuffer(file);
    } else {
      reader.readAsText(file);
    }
    console.log(file);
    return false;
}


mbody.ondragover = function() {
    return false;
};
mbody.ondragend = function() {
    return false;
};

var wemosd1=1;
var uploadimage=1;
function upload(){
      var fn=getvalue("jobname")+".gcode";
      function uploadjpg(){
	      if (uploadimage){
		c=$("prevCanvas");
		c.toBlob(function(blob){
			realupload(blob,fn+".jpg",0);
			},"image/jpeg",0.4);
	      }
      }
      realupload(new Blob([new Int8Array(compress).buffer],{type:"text/plain"}),fn,uploadjpg);
}

function download(){
        var fn=getvalue("jobname")+".gcode";
	var xhr = new XMLHttpRequest();
	xhr.responseType = "blob";
	xhr.open("GET", "http://" + getvalue("wsip") + "/" + fn, true);
	xhr.onload = function(e) {
		//alert(xhr.response);
		compress=[];
		var reader = new FileReader();
		reader.addEventListener("loadend", function() {
		    compress=new Uint8Array(reader.result);
		    decode();
		    begincompress(decodes,showProgress,aftercompress);
		});
		reader.readAsArrayBuffer(xhr.response);
//		if (wx)wxAlert("HTTP Response",s);
	};
	xhr.addEventListener('error', function(e) {
	    console.log("error"+JSON.stringify(e));
	});
	xhr.send();
}
function realupload(gcode,fname,callback) {
    var xhr = new XMLHttpRequest();
    form = new FormData();
    form.append("file1", gcode, fname);


    xhr.open(
        "POST",
        "http://"+getvalue("wsip")+"/upload",
        true
    );

	//xhr.setRequestHeader('Access-Control-Allow-Headers', '*');
    //xhr.setRequestHeader('Content-type', 'application/ecmascript');
    //xhr.setRequestHeader('Access-Control-Allow-Origin', '*');
	
    var progressBar = $('progress1');
    xhr.onload = function(e) {
        progressBar.value = 0;
        if (!wemosd1) alert(xhr.response);
		//resetflashbutton();
		if (callback)callback();
    };
    // Listen to the upload progress.
    xhr.upload.onprogress = function(e) {
        if (e.lengthComputable) {
            progressBar.value = (e.loaded / e.total) * 100;
            progressBar.textContent = progressBar.value; // Fallback for unsupported browsers.
        }
    };
    xhr.send(form);
}



var control = document.getElementById("inputfile"); 
    control.addEventListener("change", function(event){ 
        var reader = new FileReader();      
        reader.onload = function(event){
            var contents = event.target.result;  
            begincompress(contents,showProgress,aftercompress);   
            //document.getElementById('putcontentshere').value = contents;            
        };      
        reader.onerror = function(event){
            console.error("File could not be read! Code " + event.target.error.code);
        };      
        console.log("Filename: " + control.files[0].name);
        reader.readAsText(control.files[0]);        
    }, false);





var cs="";
compress=[];

window.addEventListener("load", (function() {
    var e = new URLSearchParams(window.location.search).get("view");
    null != e && ($("jobname").value = e.substr(1, e.indexOf(".") - 1),
    download())
}
), !1)
/*var editorgcode = ace.edit("gcode");
//editorgcode.setReadOnly(true);
editorgcode.session.setMode("ace/mode/gcode");
editorgcode.renderer.setShowGutter(false);
*/
//for (var i=0;i<cs.length;i++){compress.push(cs.charCodeAt(i));}
//decode();begincompress(editor.getValue());
function convertToSVG() {
            // Get the G-code input
            const gcodeInput = document.getElementById('gcodet').value;
            const from=document.getElementById('fromline').value*1;
            // Split the input into individual lines
            const lines = gcodeInput.split('\n');

            // Create SVG element
            let svg = '';
            let svgt='';
            // Set initial position to (0, 0)
            let currentX = 0;
            let currentY = 0;

            // Process each line of G-code
            var lastG=0;
            var lct=1;
            var linecnt=0;
            var pw=0;
	    var minx=miny=10000;
	    var maxx=maxy=-10000;
	    function pxm(px){return Math.round(px*377.9527559055119)*0.01;};
            lines.forEach(line => {
                linecnt++;
                // Extract G-code type (G0 or G1) and coordinates
                var ss= line.split(" ");
                var gc={};
                ss.forEach(s => {gc[s[0]]=s.substr(1);})
                if (!isNaN(gc["M"])) {
                  if (!isNaN(gc["S"]))pw=parseFloat(gc["S"])
                }
                if (!isNaN(gc["G"])) {
                    var gType,x,y;
                    gType = parseInt(gc["G"]);
                    if (lastG!=gType){
                      if (lastG==0){
                        svgt = `<polyline stroke="`
                        if (pw>200)svgt+="black";else svgt+="red";
                        svgt+=`" fill="none" points="${pxm(currentX)},${pxm(currentY)}, `;
                        lct=1;                        
                      } else {
                        svgt+='"/>';
                        if (linecnt>=from)svg+=svgt;
                      }
                      lastG=gType;
                    } else svgt+=", ";
                    if (isNaN(gc["X"]))x=currentX; else x=parseFloat(gc["X"])
                    if (isNaN(gc["Y"]))y=currentY; else y=parseFloat(gc["Y"])
                    

                    // Add SVG path
		    minx=Math.min(minx,x);
		    maxx=Math.max(maxx,x);
		    miny=Math.min(miny,y);
		    maxy=Math.max(maxy,y);
		    
                    if (gType === 1) {
                      svgt+=`${pxm(x)},${pxm(y)}`;
                      lct++;
                    }
                    // Update current position
                    currentX = x;
                    currentY = y;
                }
            });

            // Close SVG element
	    svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${maxx-minx}mm" height="${maxy-miny}mm">` +svg +'</svg>';

            // Display SVG
            document.getElementById('svgContainer').innerHTML = svg;
        }
        function downloadSVG() {
	  const svgContent = document.getElementById('svgContainer').innerHTML;
	  const blob = new Blob([svgContent], { type: 'image/svg+xml' });
	  const link = document.createElement('a');
	  link.href = URL.createObjectURL(blob);
	  link.download = 'output.svg';
	  link.click();
	}
</script>

</body>
</html>)EOF";

static const char web_configini[] PROGMEM = 
R"EOF(;=Telegram system for reporting finish job
;=Will report job name,time
;=create bot on telegram and get the token
bot_token=
;=report to this telegram ID
report_id=
;=Ip address of pc running karyacnc to report
;=default is automatically use last karyacnc IP
master=
;=connect_timeout > 0 , if timeout will create AP
;=connect_timeout = 0 , always try connect, until connected
machine_name=ESP CNC
SSID=usaha karya
connect_timeout=90
PASSWORD=45712319
;=force fixed ip
fixed_ip=192.168.1.49
;=mode router,laser,plasma
mode=router
tool_pin=D8
;=pwm multiplier
lscale=1
fscale=1
min_pwm_clock=0
;=some old pcb doesnot have reset pin
;=type= NK1661 NK1202 OLED1306 LCD2004 ST7565 ST7735
lcd_type=NK1661
lcd_addr=120
;=lcd_sda=TX
;=SPI with A0 need sda2, for st7565 D1
lcd_sda2=?
;=lcd_scl=RX
;=no need, simulated using SCL
lcd_cs=?
;=new pcb dont need rst (its simulated using SDA)
;=5 cable pcb NK1661 using D1, for st7565 ?
lcd_rst=D1
;=Only for ST7565
lcd_contrast=7
;=IR_KEY pin,  OLED1306, and LCD2004 must not same with TX
;=ir_pin=TX
;=Motor Configuration
;=motor  DIR,STEP
motor_pin=X,D5,D6
motor_pin=Y,D7,D4
;=motor_pin=Z,D0,D3
;=motor_pin=R,D7,D4
;=M,step/mm,feedmax,accel,backlash
motor=X,160,100,400,0
motor=Y,160,60,200,0
motor=Z,800,20,100,0
motor=R,80,100,100,0
;=Motor Y skew in mm per 1 Meter X
skew_y=0
;=junction rate
corner=20
;=homing x y z  0:no home, 1:home to min, >1:home to max
limit_pin=-
home=0,0,0
home_feed=50
;=thc Y/N sensor use A0 data range 0-1000
thc=-
thc_up=400
thc_ofs=100
;=only for laser, sensor, water Y/N
;=Temp sensor using DS18B20
buzzer=-
water=-
temp_pin=-
temp_limit=55
;=makezeropoint, on laser and router, put hole on 0,0
makezeropoint=N
;=PWM pin can same with tool pin	
;=if different ToolPin will only High/Low, then PWM_pin can define the tool power (such laser)
pwm_pin=-
;=laser_on,plasma_on,trimmer_on Low or High, default High
testlaser=1000
laser_on=H
plasma_on=H
trimmer_on=H
;=delay on motor step pulse
stepdelay=5
;=showkey , show remote key ID on screen
showkey=N
;=allow 2468 key to move axis XY when running job, always allow when pause
allowxyjog=N
;=Input Shaping, ZV,ZVD,MZV,EI
input_shaping=MZV,50,0.1
lcd_type=ST7565
lcd_sda=TX
lcd_sda2=D1
lcd_scl=RX
lcd_cs=?
lcd_rst=TX
ir_pin=TX
)EOF";
// lcd_sda=D3
// lcd_sda2=D1
// lcd_scl=D2
// lcd_cs=?
// lcd_rst=D3
// ir_pin=D3
// )EOF";


#endif
