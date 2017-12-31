/'
Implemented:
1. head tail buffer move
2. safespeed check for max speed
3. pathplanner

work in progress
1. pick slowest acceleration
2. jerk on corner
3. step/mm
'/
screen 12
const numaxis=3
const numbuffer=20
type tmove
    as integer totalstep,fastaxis
    as double fx(1 to numaxis),cx(1 to numaxis),sx(1 to numaxis),x1(1 to numaxis),x2(1 to numaxis),dx(1 to numaxis),odx(1 to numaxis)
    as double  stepmm,fs,fn,fe,ac1,ac2
    as integer rampup,rampdown,status       
    as integer planstatus,col
end type

type tmotor
    as integer enable
    declare sub stepping(dx as integer)
end type
sub tmotor.stepping(dx as integer)
end sub
    
dim shared as integer head,tail
dim shared as double maxf(1 to numaxis)={100,100,100}
dim shared as double stepmmx(1 to numaxis)={45,45,45}
dim shared move(1 to numbuffer) as tmove
dim shared as double fmax,jerk,accel
dim shared as double x1,y1,z1,lf
accel=0.5
jerk=35



head=1
tail=1

function nextbuff(a as integer) as integer
    a=a+1
    if a>numbuffer then a=1
    nextbuff=a
end function    
function prevbuff(a as integer) as integer
    a=a-1
    if a<1 then a=numbuffer
    prevbuff=a
end function    
function bufflen() as integer
    dim a as integer
    a=head-tail
    if a<0 then a=a+numbuffer
    bufflen=a
end function


sub safespeed(mov as tmove pointer)
    with *(mov)
        dim as integer i
        dim scale as double
        scale=1
        for i=1 to numaxis
            if .dx(i)>0 then
                .fx(i)=.fn*.dx(i)/.totalstep
                ''print .fx(i)
                dim as double scale2
                scale2=maxf(i)/.fx(i)
                if scale2<scale then scale=scale2
            end if
        next
        '' update all speed
        .fn=.fn*scale
        for i=1 to numaxis
            .fx(i)=.fx(i)*scale*.sx(i)
        next
        
    end with
end sub

sub prepareramp(mov as tmove pointer)
    with *(mov)
        color .col
        if .planstatus=1 then exit sub
        .ac1=accel/.stepmm
        .ac2=.ac1
        if .fn<.fs then .ac1=-.ac1 
        if .fn>.fe then .ac2=-.ac2 

        .rampup=(.fn-.fs)/(.ac1)
        .rampdown=(.fe-.fn)/(.ac2)

        ''print .rampup,.rampdown,.totalstep,.fs,.fn,.fe
        if .rampup+.rampdown>.totalstep then
            'print "Recalc"
            ' unable to read nominal speed
            if .rampdown>.totalstep then
                if .fs<.fn then .rampdown=(.fe-.fs)/(.ac2)
                if .rampdown>.totalstep then
                    .ac2=(.fe-.fs)/.totalstep
                    .rampup=0
                    .rampdown=.totalstep
                    ''print "x"
                else
                    .fn=.fe-.ac2*.rampdown
                    .rampup=0
                ''print "y"
                end if
            elseif .rampup>.totalstep then
                .fn=.fs+.ac1*.totalstep
                .fe=.fn
                .rampdown=0
                ''print "z"
            else
                .rampup=.totalstep-.rampdown
                .fn=.fs+.ac1*.rampup
                .rampdown=(.fe-.fn)/(.ac2)
                ''print "w"
            end if    
                
            ''print .rampup,.rampdown
        else
            ''print "OK",.rampup,.rampdown,.fs,.fn,.fe
        end if
        .planstatus=1
        ''print "cek",.fs,.fn,.fe
        
    end with
end sub
sub backplanner(h as integer)
    ' mengubah semua cross feedrate biar optimal secepat mungkin
    if bufflen()<2 then exit sub
    dim as integer i
    dim curr as tmove pointer
    dim prev as tmove pointer
    dim p as integer
    p=prevbuff(h)
    prev=@move(p)
    curr=@move(h)
    /' calculate jerk
        *        max_jerk
        *   x = -----------
        *        |v1 - v2|
        *
        *   if x > 1: continue full speed
        *   if x < 1: v = v_max * x
    '/
    dim as double scale,scale2
    scale=1
    for i=1 to numaxis
        scale2=jerk/abs(*(curr).fx(i)-*(prev).fx(i))
        if scale2<scale then scale=scale2
    next

    if *(curr).status=1 and *(prev).status<>0 then 
        ' remove prev rampdown, curr rampup
        *(prev).fe=scale* *(prev).fn''(*(prev).fn+*(curr).fn)/2
        ''*(prev).fe=scale*(*(curr).fn)
        prepareramp(prev)
        ''update current move again
        *(curr).fs=*(prev).fe
    end if
    
end sub
sub forwardplanner()
        
end sub


sub addmove(f as integer,x2 as integer ,y2 as integer ,z2 as integer)
    dim mv as tmove pointer
    
    mv=@move(nextbuff(head))
    with *(mv) 
        .col=1+(head and 7)
        .x1(1)=x1*stepmmx(1)
        .x1(2)=y1*stepmmx(2)
        .x1(3)=z1*stepmmx(3)
        .x2(1)=x2*stepmmx(1)
        .x2(2)=y2*stepmmx(2)
        .x2(3)=z2*stepmmx(3)
        .fn=f
        .fe=0
        .fs=0
        .planstatus=0 '0: not optimized 1:fixed
        .status=1 ' 0: finish 1:ready 2:running
        ' calculate delta
        dim ix as integer
        for ix=1 to numaxis
            .odx(ix)=(.x2(ix)-.x1(ix))
            .dx(ix)=abs(.odx(ix))
            .sx(ix)=1
            if .odx(ix)<0 then .sx(ix)=-1
            .cx(ix)=0
        next    
        for ix=1 to numaxis 
            if (.dx(ix)>=.dx(1)) and (.dx(ix)>=.dx(2)) and (.dx(ix)>=.dx(3))  then 
                .fastaxis=ix
            end if
        next
        .stepmm=stepmmx(.fastaxis)
        .totalstep=.dx(.fastaxis)
        safespeed(mv)
        '' next buffer please
        
        '' back planner
        head=nextbuff(head)
        backplanner(head)
    end with
    x1=x2
    y1=y2
    z1=z2
end sub

dim shared currmove as tmove pointer
dim shared tick as double
tick=0

sub gogo(mv as tmove)
    '' calculate movement
    with mv
    
        prepareramp(@mv)
        .status=2
        dim as double f,dly,x(1 to numaxis),speed
        x(1)=.x1(1)
        x(2)=.x1(2)
        x(3)=.x1(3)
        ''print .fs,.fn,.fe,.rampup,.rampdown
        f=.fs
        
        dim i as integer
        dim as double dd1,dd2,dly1,dly2,dly3
        if .fs=0 then .fs=1
        dly1=1/(.fs*.stepmm+1)
        dly2=1/(.fn*.stepmm+1)
        dly3=1/(.fe*.stepmm+1)
        dd1=0
        dd2=0
        if .rampup>0 then dd1=(dly2-dly1)/.rampup
        if .rampdown>0 then dd2=(dly3-dly2)/.rampdown
        for i=0 to .totalstep-1
            'acceleration
            dim c as integer
            c=12
            if i<.rampup then
                f=f+.ac1
                dly1=dly1+dd1
            elseif i>.totalstep-.rampdown then
                f=f+.ac2
                dly1=dly1+dd2
                c=13
            else 
                f=.fn
            end if
            speed=f
            dly=1/(f*.stepmm)
            if (i and 255)=0 then
                ''locate 20,i/80
                ''print dly2
            end if
            
            tick=tick+dly1/20
            c=.col
            pset (10*tick+10,400-speed*2),c
            pset (10*tick+10,450-dly),c
        
            pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
            dim ix as integer
            for ix=1 to numaxis
                .cx(ix)=.cx(ix)+.dx(ix)
                if (.cx(ix)>=.totalstep) then
                   x(ix)=x(ix)+.sx(ix)
                   .cx(ix)=.cx(ix)-.totalstep
                end if 
            next
        next
        .status=0
    end with
end sub
function startmove() as integer
    if (head<>tail) then
        currmove=@move(nextbuff(tail))
        *(currmove).status=2
        gogo(*(currmove))
        tail=nextbuff(tail)
        startmove=1
    else
        startmove=0
    end if
end function
sub waitbuffer()
    do
    loop until startmove()=0
end sub
x1=0
y1=0
z1=0
print "Simple Motion Control with Acceleration, Jerk, and lookahead planner"
print "By ryannining@gmail.com"

/'
addmove(500,50,100,0)
addmove(600,160,110,0)
addmove(750,180,120,0)
addmove(50,200,10,0)

addmove(10,50,50,10)
addmove(30,250,150,10)
addmove(50,250,250,10)
addmove(30,50,150,10)
'/

/'
addmove(50,300,100,0)
addmove(50,50,200,0)

dim i as integer
for i=11 to 15
    dim r as double
    r=100-i
    addmove(30,-cos(degtorad(i*20+2))*r*3+300,-sin(degtorad(i*20+2))*r+100,0)    
    if bufflen()>3 then startmove()
next
'/
addmove(150,300,100,0)
addmove(50,50,200,0)

line (0,400)-(600,400),1
line (0,450)-(600,450),2

waitbuffer()
print "Time:",tick


sleep
