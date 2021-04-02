var CMD_3D=4;
var CMD_CNC=3;
var CMD_LASER=1;
var OVC_MODE = 0; // 1 drill 0 path
function $(id) {
    return document.getElementById(id);
}
sqrt = Math.sqrt;
sqr = function(x) {
    return x * x;
}

function log(text) {
    $('log').value += text + '\n';
}

function getvalue(el) {
    return $(el).value;
}

function setvalue(el, val) {
    $(el).value = val;
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


var div = ""; //= $('mytext');
var area_dimension = $('area_dimension');
var div1; // = $('mytext1');

var text1;

function mround(x) {
    return parseFloat(x).toFixed(2);
}
var X1 = 0;
var Y1 = 0;
var lenmm = 0;
var lenn=0;
var lastf = 0;
var x1 = 0;
var y1 = 0;
var z1 = 0;
var e1 = 0;
var cmd = 0;
var xmin = 100000;
var ymin = 100000;
var xmax = 0;
var ymax = 0;

var xmin2 = 100000;
var ymin2 = 100000;
var xmax2 = 0;
var ymax2 = 0;

var sxmin = 100000;
var symin = 100000;
var sxmax = 0;
var symax = 0;
var calcmax = 0;

var gcodes = [];
var harga = 1000;
var cncz = 0;

var layerheight = 0.5;
var filamentD = 1.75;
var filamentTemp=180;
var retract = 0;
var overlap = 15;
var extrudeMul=1.5;
var extrude = (layerheight * layerheight) / (filamentD * filamentD);

var fm=0;
function gcoden(g, f, x2, y2, z2 = 10000, e2 = 1000000) {
    var xs = 1;
	g=g*1;
	f=mround(f);
	if (fm!=f){
		fm=f;
		f=" F"+f;
	} else f="";
    if (cmd == 4) xs = -1;
    div = div + 'G' + g + f + ' X' + mround(x2 * xs) + ' Y' + mround(y2);
    if (z2 != 10000) div += " Z" + mround(z2);
    if (e2 != 1000000) div += " E" + mround(e2);
    div += '\n';
    x1 = x2;
    y1 = y2;
    if (z2 != 10000) z1 = z2;
    if (e2 != 1000000) e1 = e2;
    lastf = f;
    xmin = Math.min(xmin, x2);
    ymin = Math.min(ymin, y2);
    xmax = Math.max(xmax, x2);
    ymax = Math.max(ymax, y2);
    xmin2 = Math.min(xmin2, x2);
    ymin2 = Math.min(ymin2, y2);
    xmax2 = Math.max(xmax2, x2);
    ymax2 = Math.max(ymax2, y2);
}

var inretract = 0;

function gcode0(f, x2, y2, z2 = 10000, e2 = 1000000) {
    if (retract && (cmd == 4)) {
        if (!inretract) {
            inretract = 1;
            div += "G1 F60 E" + mround(e1 - 1) + "\n";
        }
    }
    gcoden(0, f, x2, y2);
}
function gcode1(f, x2, y2, z2 = 10000, e2 = 1000000) {
    x1 -= x2;
    y1 -= y2;
    // if 3d mode, then calculate the E
    lenn = sqrt(x1 * x1 + y1 * y1);
    if (cmd == 4) {
        if (inretract) {
            div += "G1 F60 E" + mround(e1) + "\n";
            inretract = 0;
        }
        e2 = e1 + lenn * extrude*extrudeMul;
    }
    lenmm += lenn;
    gcoden(1, f, x2, y2, z2, e2);
}
var sgcodes = [];

/*

lines = [[f,x,y,len],...]

len is total length until this point

*/

function isClockwise(poly, px = 1, py = 2) {
    var sum = 0;
    for (var i = 0; i < poly.length - 1; i++) {
        var cur = poly[i];
        var next = poly[i + 1];
        sum += (next[px] - cur[px]) * (next[py] + cur[py]);
    }
    return sum > 0;
}
var overcut=[0,0];
var cross=0;
function sharp(poly, px = 1, py = 2, idx = 0, num = 2) {
    var sum = 0;
    ci = idx;
    nci = ci + 1;
    pci = ci - 1;
    if (nci >= poly.length) nci -= poly.length;
    if (pci < 0) pci += poly.length;
    prev = poly[pci];
    cur = poly[ci];
    next = poly[nci];

    vec1 = [prev[px] - cur[px], prev[py] - cur[py]];
    vec2 = [next[px] - cur[px], next[py] - cur[py]];



    d1 = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1]);
    d2 = sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1]);
    vec1 = [vec1[0]/d1, vec1[1]/d1];
    vec2 = [vec2[0]/d2, vec2[1]/d2];
    dot = (vec1[0] * vec2[0] ) + (vec1[1]  * vec2[1] );
    cross = (vec1[0] * vec2[1] ) - (vec2[0]  * vec1[1] );

	// corner overcut
	
    if (cross<0)
    overcut = [vec1[0]+vec2[0], vec1[1]+vec2[1]];
	else overcut=[0,0];
	//overcut=[cur[px]-vec3[0]/4,cur[py]-vec3[1]/4];
	

    return (dot);
}
var jmltravel = 0;


// hole inside always clockwise, outer path are counter clockwise
// to check if some hole belongs to which outer path, need to check bounding box each hole
var srl = [];
var disable_ovc=[];
var disable_tab=[];
var disable_cut=[];
var pause_at=[];
var shapectr=0;
var collapsepoint=7;
var noprocess=2*collapsepoint;
function prepare_line(lenmm,lines) {
	shapectr++;
	newlines=[];
	srl=[];
	lx=0;
	ly=0;
	if (disable_ovc.indexOf(shapectr+"")>=0)return lines;
    if (lenmm<collapsepoint){
		var sx=0;
		var sy=0;
		for (var i = 0; i < lines.length; i++) {			
			sx += lines[i][1];
			sy += lines[i][2];
		}
		sx/=lines.length;
		sy/=lines.length;
		newlines.push([lines[0][0],sx,sy,0,0]);
		return newlines;
	} else {
		if (lenmm<noprocess)return lines;
		sharpv = $("sharp").value;
		ov = $("overcut").value/10.0+0.01;
		for (var i = 0; i < lines.length; i++) {
			
			tl = lines[i][3];
			tl2 = lines[i][4];
			x = lines[i][1];
			y = lines[i][2];
			//if (ov!=0){
				sr = sharp(lines, 1, 2, i);
				if ((sr > sharpv) || (i == 0) || (i == lines.length - 1)) {
					//if (sqrt(sqr(x-lx)+sqr(y-ly))>20){
					newlines.push(lines[i]);
					ox=overcut[0]*ov;
					oy=overcut[1]*ov;
					if ((OVC_MODE==0) && ((ox!=0) || (oy!=0))) {
						//newlines.push(lines[i]);
						newlines.push([lines[i][0],x-ox,y-oy,tl,0]);
						newlines.push([lines[i][0],x,y,tl,0]);
					}
					srl.push([x, y, sr, tl, i,[x-ox,y-oy],cross]);
					lx=x;
					ly=y;
					//}
				} else newlines.push(lines[i]);
		}
		return newlines;

	}
}
var cutevery=300;

function draw_line(num, lcol, lines,srl,dash,len) {
    //if (sxmax < sxmin);
	cuttabz=1;
    var len = Math.abs(len);
    if (len < 50) cuttabz = 0;
    if (disable_tab.indexOf(num+"")>=0) cuttabz = 0;
    var lenc = 0;
	var ccw=0;
    if ($("cutccw").checked) ccw = 1;

    dv=Math.round(len/cutevery)*2;
	lc = len / dv;
    slc = lc / 2;

	
    var dpm = 440.0 / (sxmax);
    dpm = Math.min(dpm, 440.0 / (symax));
    var x = lx / dpm;
    var y = ly / dpm;
    var cxmin = 100000;
    var cymin = 100000;
    var cxmax = 0;
    var cymax = 0;
    var n = 0;
    var sc = 1;
    if ($("flipx").checked) sc = -1;
    var ro = 1;
    if ($("rotate").checked) ro = -1;
    var c = $("myCanvas1");
    //alert(c);
    var ctx = c.getContext("2d");
    ctx.font = "8px Arial";
    var g = 0;
    var X1 = 0;
    var Y1 = 0;
    cmd = getvalue('cmode');
    
    tl = 0;

    seg = $("segment").checked;

    start = 0;
    sharpv = $("sharp").value;
    ov = $("overcut").value/10.0;
    lsx = -10;
    lsy = 0;
	var i=0;
    var incut = 0;
    var lenctr = 0;
	var iscut = 0;
	ctx.beginPath();
	ctx.setLineDash([1,4]);
	ctx.moveTo(lx, ly);
	lx2=lx;
	ly2=ly;
	nc=2;
    for (var ci = 0; ci < lines.length; ci++) {
		if (ccw){
			i=(lines.length-1)-ci;}
		else {
			i=ci;
		}
        nlenctr = lenctr;
		lenctr += lines[i][4];
        //vary speed on first few mm
        x = lines[i][1];
        y = lines[i][2];
        if (sc == -1) {
            x = sxmax - x;
        }
        if (ro == -1) {
            xx = x;
            x = y;
            y = sxmax-xx;
        }
		if (cmd == CMD_CNC) {
			if (lines[i][4]){
				iscut=0;
				if (cuttabz) {
					// if cut1 is in lenc and lencnext then cut the line
					// and dont increase the i counter
					if ((slc >= lenc) && (slc <= lenctr)) {
						// split lines
						iscut = 1;
						dx = x - lx2;
						dy = y - ly2;
						llen = lenctr-lenc;

						lcut = slc - lenctr;
						x = x + dx * (lcut / llen);
						y = y + dy * (lcut / llen);
						lenc = slc;


						if ((incut == 1)) {
							incut = 0;
							slc+=lc;
						} else {
							incut = 1;
							slc += cuttablen;
						}
					}
				
				}
			}
		}
		
        g = ci;
        if (g >= 0) {
            if (g == 0) {
                X1 = x * dpm;
                Y1 = y * dpm;
                jmltravel++;
                ctx.strokeStyle = "#88888888";
            }
            if (g > 0) ctx.strokeStyle = lcol;
			if (iscut) {
				if (incut == 1) {
					// move up
				} else {
					// move back down
				}
				ci--;
				lenctr -= lines[i][4];
			} else {
				lenc = lenctr;
			}
			
            cxmin = Math.min(cxmin, x);
            cymin = Math.min(cymin, y);
            cxmax = Math.max(cxmax, x);
            cymax = Math.max(cymax, y);
			lx = x * dpm;
            ly = y * dpm;

			
			ctx.lineTo(lx, ly);
		
			lx2 = x;
			ly2 = y;
			if (g==0){
				ctx.stroke();
				ctx.beginPath();
				ctx.setLineDash(dash);
				ctx.moveTo(lx, ly);
			}
			if (iscut) {
				ctx.arc(lx,ly,2,0,2*Math.PI);
				nc+=2;
				ctx.moveTo(lx, ly);
			}
        }
    }

	ctx.lineTo(X1, Y1);
	ctx.stroke();
    d1 = sqrt(sqr(lx - X1) + sqr(ly - Y1)) / dpm;
    ctx.beginPath();
	ctx.setLineDash([2]);
    ctx.moveTo(lx, ly);
    //ctx.lineTo(X1, Y1);
    ctx.stroke();
    //ctx.endPath();
    srl.push([X1, Y1, 0, tl + d1, 0,[0,0],0]);
    //if (seg) {
        ctx.font = "10px Arial";
        sg = "[#" + num + "] ";
        for (i = 0; i < srl.length - 1; i++) {
            if (i) sg += ",";

            ctx.beginPath();
            ctx.strokeStyle = getRandomColor();
			oc=srl[i][5];
			sx=srl[i][0]*dpm;
			sy=srl[i][1]*dpm;
			
            if (srl[i][6]<0)
			{
				ctx.arc(sx, sy, 3, 0, 2 * Math.PI);
//				ctx.moveTo(sx, sy);
//				ctx.lineTo(oc[0]*dpm, oc[1]*dpm);
			}
            ctx.stroke();
			if (seg) 
			{
				ctx.fillStyle = "#0000cc";
				ni = i + 1;
				if (ni >= srl.length) ni = 0;
				ti = Math.floor((srl[i][4] + srl[ni][4]) / 2);
				l = mround(srl[ni][3] - srl[i][3]);
				sg += l;
				ctx.fillText(l, (lines[ti][1] * dpm) + 10, (lines[ti][2] * dpm) + 10);
			}
		}
        if (seg)$("segm").value += sg + "\n";
    //}
    //+" W:"+mround(cxmax-cxmin)+" H:"+mround(cymax-cymin)+" "
	clw="<";
	if (isClockwise(lines))clw=">";
    if (cxmin < cxmax) ctx.fillText("#" + num+clw, dpm * ((cxmax - cxmin) / 2 + cxmin), dpm * cymax + 10);
}
var lastz = 0;
var lasee = 0;
var lx;
var ly;
var cuttablen = 10;
var lastspeed = 0.6;
var tabcutspeed = 0.75;
function lines2gcode(num, data, z, cuttabz, srl,lastlayer = 0,firstlayer=1) {
    // the idea is make a cutting tab in 4 posisiton,:
    //
    var len = Math.abs(data[0]);
    if (len < 50) cuttabz = z;
    if (disable_tab.indexOf(num+"")>=0) cuttabz = z;
    var lenc = 0;
	var ccw=0;
    if ($("cutccw").checked) ccw = 1;

    dv=Math.round(len/cutevery)*2;
	lc = len / dv;
    slc = lc / 2;


	fm=0;
    var lines = data[4];
    var X1 = lines[0][1];
    var Y1 = lines[0][2];
    var sc = 1;
    if ($("flipx").checked) {
        sc = -1;
        X1 = sxmax - X1;
    }
    var ro = 1;
    if ($("rotate").checked) {
        ro = -1;
        XX = X1;
        X1 = Y1;
        Y1 = sxmax-XX;
    }
	if (!(lx===undefined)){
		pdis=sqrt(sqr(lx-X1)+sqr(ly-Y1)); 
	} else pdis=0;
	
    lx = X1;
    ly = Y1;

    // turn off tool and move up if needed
    cmd = getvalue('cmode');
    var pw1 = 1;
    var pw2 = 0; //getvalue('pwm');
    var pup = getvalue('pup');
    var pdn = getvalue('pdn');
    var f1 = getvalue('trav') * 60;
    var f2 = getvalue('feed') * 60;
	var rep=getvalue('repeat')*1;
	if (rep>1){
		if (lastlayer) {
			f1*=lastspeed;
			f2*=lastspeed;
		} else {
			if (cuttabz > z) {
				f1*=tabcutspeed;
				f2*=tabcutspeed;
			}
		}
	}
    if (cmd == 2) {
        pw1 = pw2;
        f1 = f2;
    }
    div = "";
    if (sxmax < sxmin) return cdiv;
    // deactivate tools and move to cut position
    div = div + "\n;SHAPE #" + num + "\n";
    if (cmd == CMD_3D) {
        z = -z; // if 3D then move up
        if (z <= layerheight) {
            f2 = 800;
            extrude = 3 * (layerheight * layerheight) / (filamentD * filamentD);

        } else {
            extrude = 2 * (layerheight * layerheight) / (filamentD * filamentD);
        }
		// extra from after travel
		div+=";extra feed\nG92 E"+mround(e1-(pdis/350.0))+"\n";
    }
    var oextrude = extrude;


    if (pw2) div = div + "M106 S" + pw1 + "\n";
    if (firstlayer)div = div + pup + '\n';
    if (cmd == 2) {
        gcode0(f1, X1, 0);
    }
    gcode0(f1, X1, Y1);

    //if (cmd == 3) div = div + "G0 Z" + mround(lastz) + "\n";

    // activate tools and prepare the speed
    if (pw2) div = div + "M106 S" + pw2 + "\n";
    if (cmd != CMD_3D) div = div + pdn.replace("=cncz", mround(z)) + '\n';
    var incut = 0;
    var lenctr = 0;
    var fm = 1;
	if (cmd==CMD_CNC){
		// entry diagonally
		// ??
	
	}		
		
    for (var ci = 0; ci < lines.length; ci++) {
		if (ccw){
			i=(lines.length-1)-ci;}
		else {
			i=ci;
		}
        nlenctr = lenctr;
		lenctr += lines[i][4];
        //vary speed on first few mm
        if (cmd == CMD_3D) {
			nlenctr=(nlenctr+lenctr)/2;
            extrude = oextrude;
            fm = 1;
			
            if (nlenctr <= 20) {
                fm = Math.ceil(100 * (nlenctr + 2) / 32.0) / 100.0;
                extrude*=(1.5-fm);
            } else if (len - nlenctr < 10) {
                // reduce extrude before few last mm
                if (!lastlayer)extrude *= 0.85;
            }
        }
        x = lines[i][1];
        y = lines[i][2];
        if (sc == -1) {
            x = sxmax - x;
        }
        if (ro == -1) {
            xx = x;
            x = y;
            y = sxmax-xx;
        }
        var iscut = 0;
		if (cmd == CMD_CNC) {
           if ((cuttabz > z)) {
            // if cut1 is in lenc and lencnext then cut the line
            // and dont increase the i counter
            if ((slc >= lenc) && (slc <= lenctr)) {
                // split lines
				iscut = 1;
				dx = x - lx;
				dy = y - ly;
				llen = lenctr-lenc;

				lcut = slc - lenctr;
				x = x + dx * (lcut / llen);
				y = y + dy * (lcut / llen);
				lenc = slc;

				//lines[i][1]=x;
				//lines[i][2]=y;
				//lines[i][3]-=lcut;

				if ((incut == 1)) {
					incut = 0;
					slc+=lc;
				} else {
					incut = 1;
					slc += cuttablen;
				}
			}
			}
		}

		if (ci==0){
			if ( firstlayer)div+=";if you want to slow down first layer do it here\n";
			if ( lastlayer)div+=";if you want to fast up last layer do it here\n";
        }
		gcode1(f2 * fm, x, y);
        lx = x;
        ly = y;
        if (iscut) {
            if (incut == 1) {
                // move up
                div = div + pdn.replace("=cncz", mround(cuttabz)) + '\n';
            } else {
                // move back down
                div = div + pdn.replace("=cncz", mround(z)) + '\n';
            }
            ci--;
			lenctr -= lines[i][4];
        } else {
            lenc = lenctr;
        }
    }
    //close loop
    if (!lastlayer && (cmd == CMD_3D)) {
        // if 3d then move along some mm
        lenctr = 0;
        lenc = 0;
        cutat = overlap;
        var dz = z - lastz;
        div = div + "; move up and overlap " + cutat + "mm\n";
        var oe1 = e1;
        extrude = oextrude * 0.4;
        for (var i = 0; i < lines.length; i++) {
            //vary speed on first few mm
            x = lines[i][1];
            y = lines[i][2];
            if (sc == -1) {
                x = sxmax - x;
            }
            if (ro == -1) {
                xx = x;
                x = y;
                y = xx;
            }
            // if cut1 is in lenc and lencnext then cut the line
            // and dont increase the i counter
            var iscut = 0;
            if ((cutat >= lenc) && (cutat <= lenctr)) {
                // split lines
                iscut = 1;
                dx = x - lx;
                dy = y - ly;
                llen = lenctr - lenc;
                lcut = cutat - lenc;
                x = lx + dx * (lcut / llen);
                y = ly + dy * (lcut / llen);
                lenc = cutat;

                //lines[i][1]=x;
                //lines[i][2]=y;
                //lines[i][3]-=lcut;

            }
            zz = lastz + dz * (lenc / cutat);
            gcode1(f2 * fm, x, y, zz);
            lx = x;
            ly = y;
            if (iscut) break;
            lenc = lenctr;
            if (lenc > cutat) break;
        }
        e1 = oe1;
        // if not 3d	
    } else gcode1(f2 * fm, X1, Y1);


    if (cmd == 2) {
        // if foam mode must move up
        gcode0(f1, X1, 0);
    }
    //gcode0(f1,X1,Y1);
    lastz = z;

    return div;
}

function getRandomColor() {
    var letters = '0123456789ABCDEF';
    var color = '#';
    for (var i = 0; i < 6; i++) {
        color += letters[Math.floor(Math.random() * 16)];
    }
    return color;
}

function gcode_verify() {
    var c = $("myCanvas1");
    cmd = getvalue('cmode');
    harga=getvalue('matprice');
	var hargacut=getvalue('cutprice');
	//alert(c);
    lx = 0;
    ly = 0;
    jmltravel = 0;
    var ctx = c.getContext("2d");
    var sfinal = 0;
    $("segm").value = "";
    ctx.clearRect(0, 0, c.width, c.height);
    for (var i = 0; i < sgcodes.length; i++) {
		col=getRandomColor();
        if (disable_cut.indexOf(sgcodes[i][1]+"")>=0)
		{
			dash=[5, 5];
			col="#FF0000";
		} else {
			dash=[];
			sfinal += Math.abs(sgcodes[i][0][0]);
		}			
			
		draw_line(sgcodes[i][1], col, sgcodes[i][0][4],sgcodes[i][0][5],dash,sgcodes[i][0][0]);
		
    }
    //sfinal+=jmltravel*10;
    ctx.font = "12px Arial";
    w = mround((xmax - xmin) / 10);
    h = mround((ymax - ymin) / 10);
    ctx.fillText("W:" + w + " H:" + h + " Luas:" + mround(w * h) + " cm2", 0, c.height - 20);
    var menit = mround((sfinal + jmltravel * 10) / getvalue('feed') / 60.0);
    var re = getvalue("repeat");
    menit = menit * re;
    text = $("material");
    mat = text.options[text.selectedIndex].innerText;
	if (cmd==CMD_3D){
		gram=e1/333;
		
		area_dimension.innerHTML = 'Total filament =' + mround(gram) + "gram Time:" + mround(menit) + " menit <br>Biaya Total:" + Math.round(gram * hargacut);
	} else {
		area_dimension.innerHTML = 'Total Length =' + mround(sfinal) + "mm Time:" + mround(menit) + " menit <br>Biaya Cut:" + Math.round(menit * hargacut) + " bahan (" + mat + "):" + mround(w * h * harga) + " TOTAL:" + mround(menit * hargacut + w * h * harga);
	}
}

function sortedgcode() {
    sgcodes = [];
    sxmax = xmax;
    symax = ymax;
    sxmin = xmin;
    symin = ymin;
    xmax = -10000;
    ymax = -10000;
    xmin = 100000;
    ymin = 100000;
    var sm = -1;
    var lx = 0;
    var ly = 0;
    e1 = 0;
	sortit=1;
    for (var j = 0; j < gcodes.length; j++) {
		if (sortit){
			var cs = -1;
			var bg = 10000000;
			for (var i = 0; i < gcodes.length; i++) {

				var dx = gcodes[i][2] - lx;
				var dy = gcodes[i][3] - ly;
				var dis = sqrt(dx * dx + dy * dy) + gcodes[i][6];
				if ((gcodes[i][6] > 0) && (dis < bg)) {
					cs = i;
					bg = dis;
				}
			}
		} else cs=j;
        // smalles in cs
        if (cs >= 0) {
            sgcodes.push([gcodes[cs],cs+1]);
            gcodes[cs][6] = -gcodes[cs][6];
            lx = gcodes[cs][2];
            ly = gcodes[cs][3];
        }
    }
    if (cmd == 4) setvalue("repeat", Math.ceil(getvalue("zdown") / layerheight));
    var re = getvalue("repeat");
    var ov = getvalue("overcut")*1.0;
    s = ""; //;Init machine\n;===============\nM206 P80 S20 ;x backlash\nM206 P84 S20 ;y backlash\nM206 P88 S20 ;z backlash\n;===============\n";
    pup1=getvalue("pup");
	cncdeep0 = -getvalue("zdown");
	pdn1 = getvalue("pdn").replace("=cncz", mround(cncdeep0)) + '\n';
    cncdeep = cncdeep0;
    var cuttab = 0;
    lastz = 0;

    cmd = getvalue('cmode');
    if (cmd == CMD_CNC) {
        lastz = layerheight * 0.7;
        cncz -= lastz;
        s += "g0 f1000 z" + mround(lastz);
    }
    cuttab = cncdeep + getvalue("tabc") * 1;
	for (var j = 0; j < sgcodes.length; j++) {
		cncz = cncdeep / re;
		if (pause_at.indexOf(sgcodes[j][1]+"")>=0){
			s+="\nG0 Z3 F1000\n;PAUSE\nG0 Y-100 F5000\n";
		}
		if (disable_cut.indexOf(sgcodes[j][1]+"")<0) {
			srl=sgcodes[j][0][5];
			if ((ov>0) && (cmd == CMD_CNC)) {
				if (OVC_MODE==1){
				// drill overcut
				for (i = 0; i < srl.length - 1; i++) {
					// drill at overcut position
					if (srl[i][6]<0)
					{
						oc=srl[i][5];
						ocxy="X"+mround(oc[0])+" Y"+mround(oc[1]);
						s+="\n"+pup1+"\nG0 "+ocxy+" F5000\n";
						s+=pdn1+pup1+"\n";
					}
				}
				}
			}		
			
			for (var i = 0; i < re; i++) {
				s += lines2gcode(sgcodes[j][1], sgcodes[j][0], cncz, cuttab,sgcodes[j][0][5],i==re-1,i==0);
				cncz += cncdeep / re;
			}
		}
    }
    s = s + getvalue("pup");
    s = s + '\nG00 F3000 Y0 \n G00 X0\n';
    sc = 1;
    if ($("flipx").checked) sc = -1;
    if (cmd == CMD_3D) {
        // make it center
        s = "g28\ng0 z10 f1000\nm109 s"+filamentTemp+"\nG92 X" + mround(-sc * xmax / 2) + " Y" + mround(ymax / 2) + " E-5\n" + s;
        s += "G92 X" + mround(sc * xmax / 2) + " Y" + mround(-ymax / 2) + "\ng28";
    }
    setvalue("gcode", s);
    setvalue("pgcode", getvalue("pup") + "\nM3 S255 P10\nG0 F10000 X" + mround(sc * xmin) + " Y" + mround(ymin) + "\nM3 S255 P10\nG0 X" + mround(sc * xmax) + "\nM3 S255 P10\nG0 Y" + mround(ymax) + "\nM3 S255 P10\nG0 X" + mround(sc * xmin) + " \nM3 S255 P10\nG0 Y" + mround(ymin) + "\n");
}

////////////////////////////////////////////////////////////////////////////////////////////


function destroyClickedElement(event) {
    document.body.removeChild(event.target);
}

var lines = [];

function xarea(){
	dx=xmax2-xmin2;
	dy=ymax2-ymin2;
	v=dx*dy;
	
	xmin2 = 100000;
    ymin2 = 100000;
    xmax2 = 0;
    ymax2 = 0;
	return v;
}
function myFunction(scale1) {
    //text1=Potrace.getSVG(1);
    //alert(text1);
	pause_at=getvalue('pauseat').split(",");
	disable_cut=getvalue('disablecut').split(",");
	disable_ovc=getvalue('disableovc').split(",");
	disable_tab=getvalue('disabletab').split(",");
	shapectr=0;
    var contor = 0;
    var xincep = 0;
    var yincep = 0;
    var p1x = 0;
    var p1y = 0;
    var p2x = 0;
    var p2y = 0;
    e1 = 0;
    var xsfar = 0;
    var ysfar = 0;
    var n1 = 0;
    lines = [];
    var scale = 25.4 / getvalue('scale');
    if (scale1) scale = 1;
    //var division = $('division').value;

    //path d="M111.792 7.750 C 109.785 10.407,102.466 13.840,100.798 12.907 C
    //$("gsvg").value=text1;
    cmd = getvalue('cmode');
    var pw1 = 1;
    var pw2 = 0; //getvalue('pwm');
    var pup = getvalue('pup');
    var pdn = getvalue('pdn');
    var f1 = getvalue('trav') * 60;
    var f2 = getvalue('feed') * 60;
    var det = getvalue('feed') / (60.0*getvalue("smooth"));
    var seg = $("segment").checked;
    if (seg) det *= 4;
    if (cmd == 2) {
        pw1 = pw2;
        f1 = f2;
    }
    //alert(cmd);

    var n = text1.indexOf(' d="M');
    n = n + 5;
    var handleM = 1;
    var X1 = 0;
    var Y1 = 0;

    xarea();
	

    gcodes = [];
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;
    div = "";
    lenmm = 0;
	lenn=0;
	
    var cnts = 0;
    var line = [];
	//scale=1;
    //alert(div.innerHTML);
    while (1) {
        if (handleM) {
            var mm = ' ';
            if (text1.charAt(n) == ' ') {
                mm = ',';
                n = n + 1;
            }
            var res = scale * parseFloat(text1.slice(n, n + 10)); // '111.792 7.'
            //res=res*scale;
            //alert(res);
            xincep = res;
            var n = text1.indexOf(mm, n + 1);
            n = n + 1; //7.750 C
            var res = scale * parseFloat(text1.slice(n, n + 10));
            //res=res*scale;
            //alert(res);
            yincep = res;
            cnts = cnts + 1;
            // close shape loop
            if (cnts > 1) {
                ///
gcode1(f2, X1, Y1);
                line.push([f2, X1, Y1, lenmm,lenn]);
                if (cmd == 2) {
                    // if foam mode must move up
                    div = div + 'G00 Y0 \n';
                }
                gcodes.push([lenmm, div, X1, Y1, prepare_line(lenmm,line),srl,xarea()]);
				
                //lines.push(line);

            }
            line = [];
            div = "";
            lenmm = 0;
			lenn=0;

            // deactivate tools and move to cut position
            div = div + "\n;SHAPE\n";
            if (pw2) div = div + "M106 S" + pw1 + "\n";
            div = div + pup + '\n';

            X1 = xincep;
            Y1 = yincep;
            ///
gcode0(f1, X1, Y1);
            // activate tools and prepare the speed
            if (pw2) div = div + "M106 S" + pw2 + "\n";
            div = div + pdn + '\n';
            div = div + 'G01 F' + mround(f2) + '\n';

            lastf = f2;
            handleM = 0;
        }
        var n = text1.indexOf(' ', n + 1);
        cr = text1.charAt(n + 1);
        if ((cr >= '0') && (cr <= '9')) {
            var n2 = text1.indexOf(' ', n + 2);
            var xy = text1.slice(n + 1, n2).split(',');
            p1x = xy[0] * scale;
            p1y = xy[1] * scale;
            ///
gcode1(f2, p1x, p1y);
            line.push([f2, p1x, p1y, lenmm,lenn]);
        } else if (cr == 'H') {
            var n2 = text1.indexOf(' ', n + 3);
            var xy = text1.slice(n + 3, n2);
            p1x = xy * scale;

            ///
gcode1(f2, p1x, y1);
            line.push([f2, p1x, y1, lenmm,lenn]);
            n = n + 3;
        } else if (cr == 'V') {
            var n2 = text1.indexOf(' ', n + 3);
            var xy = text1.slice(n + 3, n2);
            p1y = xy * scale;

            ///
gcode1(f2, y1, p1y);
            line.push([f2, y1, p1y, lenmm,lenn]);

            n = n + 3;
        } else if (cr == 'C') {
            //path d="M111.792 7.750 C 109.785 10.407,102.466 13.840,100.798 12.907 C
            var res = scale * parseFloat(text1.slice(n + 2, n + 10));
            //res=res*scale;
            //alert(res);
            p1x = res;

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //alert(res);
            p1y = res;

            var n = text1.indexOf(',', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //alert(res);
            p2x = res;

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //alert(res);
            p2y = res;

            var n = text1.indexOf(',', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //alert(res);
            xsfar = res;

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //alert(res);
            ysfar = res;

            //*****************************

            var a = p1x - xincep
            var b = p1y - yincep
            var a = sqrt(a * a + b * b);

            var b = p2x - p1x
            var c = p2y - p1y
            var b = sqrt(b * b + c * c);

            var c = xsfar - p2x
            var d = ysfar - p2y
            var c = sqrt(c * c + d * d);

            //g=1/((a+b+c)*division);
            g = det / (a + b + c);
            a = a + b + c;
            //alert('dist ='+a+' pezzi='+g);
            //******************************
            for (i = 0; i < 1; i += g) {

                var x = bezierx(i, xincep, p1x, p2x, xsfar);
                var y = beziery(i, yincep, p1y, p2y, ysfar);
                ///
gcode1(lastf, x, y);
                line.push([lastf, x, y, lenmm,lenn]);
            }

            //******************************************************************

            xincep = xsfar;
            yincep = ysfar;
        } else if (cr == 'L') { //alert("este L");

            //div.innerHTML = div.innerHTML +'(este linie) \n\n\n';
            var res = scale * parseFloat(text1.slice(n + 2, n + 10));
            //res=res*scale;
            p1x = res;
            //console.log(res);

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //console.log(res);
            p1y = res;
            ///
gcode1(lastf, p1x, p1y);
            line.push([lastf, p1x, p1y, lenmm,lenn]);

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //console.log(res);
            p2x = res;

            var n = text1.indexOf(' ', n + 3);
            var res = scale * parseFloat(text1.slice(n + 1, n + 10));
            //res=res*scale;
            //console.log(res);

            p2y = res;
            ///
gcode1(lastf, p2x, p2y);
            line.push([lastf, p2x, p2y, lenmm,lenn]);

            xincep = p2x;
            yincep = p2y;

        } else if (cr == 'M') {
            //console.log("este M");
            n = n + 2;
            handleM = 1;
        } else if (text1.slice(n + 1, n + 5) == ' d="M') {
            //console.log("este M");
            n = n + 5;
            handleM = 1;
        } else if (n < 1) {
            //console.log("lenght depasit");
            break;
        } else {
            console.log("unknown :" + text1.slice(n + 1, n + 10));
            var n = text1.indexOf(' d="M', n + 1);
            if (n < 1) break;
            n = n + 5;
            handleM = 1;
        }

    } //sfarsit while

    // close loop
    if (cnts > 0) {
        ///
gcode1(f2, X1, Y1);
        line.push([lastf, X1, Y1, lenmm,lenn]);
        if (cmd == 2) {
            // for foam, must move up first
            div = div + 'G00 Y0 \n';
        }
        gcodes.push([lenmm, "", X1, Y1, prepare_line(lenmm,line),srl,xarea()]);
        div = "";
    }
    sortedgcode();
	gcode_verify();

} //sfarsit myFunction
//*****************************************************

function bezierx(t, p0, p1, p2, p3) {
    var s = 1 - t;
    var x = s * s * s * p0 + 3 * (s * s * t) * p1 + 3 * (t * t * s) * p2 + t * t * t * p3;
    return x;
}

function beziery(t, p0, p1, p2, p3) {
    var s = 1 - t;
    var y = s * s * s * p0 + 3 * (s * s * t) * p1 + 3 * (t * t * s) * p2 + t * t * t * p3;
    return y;
}

var gc="g0 x100 y0\ng1 x200\ng1 y100\ng1 x100\ng1 y0\n";
function gcodetoText1(gx){
	// try to support G2 and G3
	gs=gx.split("\n");
	var scale = getvalue('scale')/25.4;
	sn=0;
	var c=0;
	var sc=0;
	var t1='<svg id="svg" version="1.1" width="142" height="142" xmlns="http://www.w3.org/2000/svg"><path d="';
	var tm="";			
	var wd={'g':-1,'x':0,'y':0};
	var xy1='';
	for (i in gs){
		if ((gs[i]) && (gs[i][0]!=';')){
			ws=gs[i].split(" ");
			wd['g']=-1;
			hasxy=0;
			for (j in ws){
				if (ws[j]) {
					cr=ws[j][0].toLowerCase();
					if (cr=='x' || cr=='y')hasxy=1;
					wd[cr]=ws[j].substr(1);
				}
				//console.log(ws[j][0]+":"+ws[j].substr(1));
			}
			if (hasxy){
				xy=mround(wd['x']*scale)+" "+mround(wd['y']*scale);
				if (wd['g']==0) {
					// new shape
					sc++;
					if (sc>1){
						if ((c & 1))t1+=" "+xy1;
						t1+=" ";
					}
					c=0;
					tm="M"+xy;
					xy1=xy;
				}
				if (wd['g']==1) {
					if (c==0){
						sn++;
						t1+=tm;
					}
					c++;
					if (c & 1)t1+=" L";
					t1+=" "+xy;
				}
			}
		}
	}
	if ((c & 1))t1+=" "+xy1;
	t1+='" stroke="none" fill="black" fill-rule="evenodd"/></svg>';
	return t1;
}

//***********************************************

var openFile = function(event) {
    var input = event.target;

    var reader = new FileReader();
    reader.onload = function() {
        var dataURL = reader.result;
        //var output = $('output');
        //output.src = dataURL;

    };
    reader.readAsDataURL(input.files[0]);
    Potrace.loadImageFromFile(input.files[0]);
    Potrace.process(function() {
        //displayImg();
        //displaySVG(scale);
        text1 = Potrace.getSVG(1); //.toUpperCase();
        refreshgcode();

    });

};

// handle paste image
// We start by checking if the browser supports the
// Clipboard object. If not, we need to create a
// contenteditable element that catches all pasted data
if (!window.Clipboard) {
    var pasteCatcher = document.createElement("div");

    // Firefox allows images to be pasted into contenteditable elements
    pasteCatcher.setAttribute("contenteditable", "");

    // We can hide the element and append it to the body,
    pasteCatcher.style.opacity = 0;
    document.body.appendChild(pasteCatcher);

    // as long as we make sure it is always in focus
    pasteCatcher.focus();
    document.addEventListener("click", function() {
        pasteCatcher.focus();
    });
}
// Add the paste event listener
window.addEventListener("paste", pasteHandler);

/* Handle paste events */
function pasteHandler(e) {
    // We need to check if event.clipboardData is supported (Chrome)
    if (e.clipboardData) {
        // Get the items from the clipboard
        var items = e.clipboardData.items;
        if (items) {
            // Loop through all items, looking for any kind of image

            for (var i = 0; i < items.length; i++) {

                if (items[i].type.indexOf("image") !== -1) {
                    // We need to represent the image as a file,
                    var blob = items[i].getAsFile();
                    // and use a URL or webkitURL (whichever is available to the browser)
                    // to create a temporary URL to the object
                    var URLObj = window.URL || window.webkitURL;
                    var source = URLObj.createObjectURL(blob);

                    // The URL can then be used as the source of an image
                    createImage(source, blob);
                }
            }
        }
        // If we can't handle clipboard data directly (Firefox),
        // we need to read what was pasted from the contenteditable element
    } else {
        // This is a cheap trick to make sure we read the data
        // AFTER it has been inserted.
        setTimeout(checkInput, 1);
    }
}
/*
window.addEventListener('dragover', function(e) {
    e.stopPropagation();
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
});
*/
window.addEventListener('Xdrop', function(e) {
    e.stopPropagation();
    e.preventDefault();
    var files = e.dataTransfer.files; // Array of all files

    for (var i = 0, file; file = files[i]; i++) {
        if (file.type.match(/image.*/)) {
            var reader = new FileReader();
            reader.tipe = file.type;
            reader.onload = function(event) {
                var dataURL = reader.result;
                //event.target.result;
                //var output = $('output');
                //output.src = dataURL;

                if (reader.tipe.indexOf("svg") !== -1) {
                    text1 = event.target.result; //.toUpperCase();
                    myFunction(1);
                } else {}
            };
            if (file.type.indexOf("svg") !== -1) {
                reader.readAsText(file);

            } else {
                reader.readAsDataURL(file);
                Potrace.loadImageFromFile(file);
                Potrace.process(function() {
                    //displayImg();
                    //displaySVG(scale);
                    text1 = Potrace.getSVG(1); //.toUpperCase();

                    refreshgcode();
                });
            }
        }
    }
});

/* Parse the input in the paste catcher element */
function checkInput() {
    // Store the pasted content in a variable
    var child = pasteCatcher.childNodes[0];

    // Clear the inner html to make sure we're always
    // getting the latest inserted content
    pasteCatcher.innerHTML = "";

    if (child) {
        // If the user pastes an image, the src attribute
        // will represent the image as a base64 encoded string.
        if (child.tagName === "IMG") {
            createImage(child.src);
        }
    }
}

/* Creates a new image from a given source */
function createImage(source, blob) {
    var pastedImage = new Image();
    //clear gcode
    pastedImage.onload = function() {
        Potrace.loadImageFromFile(blob);
        Potrace.info.alphamax = getvalue("smooth");
        Potrace.process(function() {
            //displayImg();
            //displaySVG(scale);
            text1 = Potrace.getSVG(1); //.toUpperCase();

            refreshgcode();

        });
    }
    pastedImage.src = source;
}

function createSVG(source, blob) {
    //displaySVG(scale);
    text1 = blob;

    refreshgcode();
}

function refreshgcode() {
    myFunction(0);
    savesetting();
}

function copy_to_clipboard(id) {
    $(id).select();
    document.execCommand('copy');
}


// Web socket server to allow other karyacnc apps send gcode and task to this software