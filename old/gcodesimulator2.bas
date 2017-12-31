/'
Implemented:
1. head tail buffer move
2. safespeed check for max speed
3. pathplanner
4. jerk safe speed on turning
5. step/mm for each axis

work in progress
1. individual axis acceleration
2. individual jerk
'/
screen 12
const numaxis=3
const numbuffer=20
type tmove
    as integer totalstep,fastaxis
    as double fx(1 to numaxis),cx(1 to numaxis),sx(1 to numaxis),x1(1 to numaxis),x2(1 to numaxis),dx(1 to numaxis),odx(1 to numaxis)
    as double  stepmm,jerk,fs,fn,fe,ac,ac1,ac2
    as integer rampup,rampdown,status       
    as integer planstatus,col
end type

type tmotor
    as integer enable
    declare sub stepping(dx as integer)
end type
sub tmotor.stepping(dx as integer)
    if dx<0 then
        
    elseif dx>0 then
    
    end if
end sub

dim shared as tmotor mymotor(1 to numaxis) 
dim shared as integer head,tail
dim shared as double jerk(1 to numaxis),accel(1 to numaxis),maxf(1 to numaxis)
dim shared as double stepmmx(1 to numaxis)={45,45,45}
dim shared move(1 to numbuffer) as tmove
dim shared as double fmax
dim shared as double x1,y1,z1,lf
accel(1)=20
accel(2)=20
accel(3)=5

jerk(1)=75
jerk(2)=75
jerk(3)=5

maxf(1)=100
maxf(2)=100
maxf(3)=5


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
        ''ioprint "w",.fx(1),.fx(2),.fx(3)
        
    end with
end sub

function ramplen(v0 as double,v1 as double,a as double,stepmm as double) as double
    dim t as double
    t=abs(v1-v0)/a
    ramplen=abs((v0*t+0.5*a*t^2)*stepmm)
end function
function speedat(v0 as double,a as double,s as double) as double
    speedat=sqr(a*2*s+v0^2)
end function
function accelat(v0 as double,v1  as double,s as double) as double
    ''v1=sqr(a*2*s+v0*v0)
    ''a=(v1*v1-v0*v0)/(2*s)
    accelat=(v1*v1-v0*v0)/(2*s)
end function

sub prepareramp(bpos as integer)
    dim mov as tmove pointer
    mov=@move(bpos)
    with *(mov)
        color .col
        if .planstatus=1 then exit sub
        print bpos
        .ac1=.ac
        .ac2=.ac
        if .fn<.fs then .ac1=-.ac1 
        if .fn>.fe then .ac2=-.ac2 
        dim t as double
        
        
        .rampup=ramplen(.fs,.fn,.ac1,.stepmm)
        .rampdown=ramplen(.fe,.fn,.ac2,.stepmm)
        
        ''print .rampup,.rampdown,.totalstep,.fs,.fn,.fe
            'print "Recalc"
            ' unable to read nominal speed
        if .rampup>.totalstep/2 then
            .fn=speedat(.fs,.ac1/.stepmm,.totalstep/2)
            .ac1=.ac
            .ac2=.ac
            if .fn<.fs then .ac1=-.ac 
            if .fe<.fn then .ac2=-.ac 
            .rampup=.totalstep/2
            .rampdown=ramplen(.fe,.fn,.ac2,.stepmm)
            ''print "x",.fn,.rampdown
        end if
        if .rampdown>.totalstep/2 then
            dim f2 as double
            if .ac2>0 then 
                .fe=speedat(.fn,.ac2/.stepmm,.totalstep/2)
            else
                f2=speedat(.fe,-.ac2/.stepmm,.totalstep/2)
                if f2<.fn then 
                    .fn=f2
                end if    
            end if
            .ac1=.ac
            .ac2=.ac
            if .fn<.fs then .ac1=-.ac
            if .fe<.fn then .ac2=-.ac
            .rampdown=ramplen(.fe,.fn,.ac2,.stepmm)
            .rampup=ramplen(.fs,.fn,.ac1,.stepmm)
            ''print "y",.fn
        end if
        .planstatus=1
        if .rampup>.totalstep then
            '' nothing work, so need to track back and reduce all speed along the way
            '' for now, just maximize acceleration, force it
            .ac1=accelat(.fs,.fe,.totalstep)*.stepmm
            .ac2=0
            .rampup=.totalstep
            .rampdown=0
            ''print "cek",
        end if
    end with
end sub
sub planner(h as integer)
    ' mengubah semua cross feedrate biar optimal secepat mungkin
    if bufflen()<2 then exit sub
    dim as integer i
    dim curr as tmove pointer
    dim prev as tmove pointer
    dim p as integer
    p=prevbuff(h)
    prev=@move(p)
    curr=@move(h)
    safespeed(curr)
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
        scale2=*(curr).jerk / abs(*(curr).fx(i)-*(prev).fx(i))
        if scale2<scale then scale=scale2
    next

    if *(curr).status=1 and *(prev).status<>0 then 
        ' remove prev rampdown, curr rampup
        *(prev).fe=scale* *(curr).fn''(*(prev).fn+*(curr).fn)/2
        ''print *(prev).fe,scale
        ''*(prev).fe=scale*(*(curr).fn)
        prepareramp(p)
        ''update current move again
        *(curr).fs=*(prev).fe
    end if
    
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
        .jerk=jerk(.fastaxis)
        .ac=accel(.fastaxis)
        .stepmm=stepmmx(.fastaxis)
        .totalstep=.dx(.fastaxis)
        
        '' next buffer please
        
        '' back planner
        head=nextbuff(head)
        planner(head)
    end with
    x1=x2
    y1=y2
    z1=z2
end sub

dim shared currmove as tmove pointer
dim shared as double tick,tickscale,fscale
dim shared x(1 to numaxis) as double
tick=0
tickscale=5
fscale=3
x(1)=0
x(2)=0
x(3)=0
x1=0
y1=0
z1=0

sub gogo(bpos as integer)
    dim mv as tmove pointer
    mv=@move(bpos)
    '' calculate movement
    with *mv
        ''prepareramp(bpos)
        ''print .rampup,.rampdown,.totalstep
        .status=2
        dim as double f,dly,speed
        ''print .fs,.fn,.fe,.rampup,.rampdown
        f=.fs
        ''x(1)=.x1(1)
        ''x(2)=.x1(2)
        ''x(3)=.x1(3)
                
        dim i as integer
        dim as double dly2,tac1,tac2 
        if .fs=0 then .fs=1
        dly2=1000/.fs
        tac1=0.001/.ac1
        tac2=0.001/.ac2
        
        dim dl as double
        ''print f,.ac1,.totalstep
        for i=0 to .totalstep
            if f>0 then 
                dl=1/(f*.stepmm)
            else 
                dl=1/.stepmm
            end if
            'acceleration
            dim c as integer
            c=12
            
            tick=tick+dl
            c=.col
            pset (tick*tickscale+10,400-f*fscale),c
        
            pset (x(1)/stepmmx(1)+50,x(2)/stepmmx(2)+40),c
            dim ix as integer
            '' bresenham work on motor step
            for ix=1 to numaxis
                .cx(ix)=.cx(ix)+.dx(ix)
                if (.cx(ix)>=.totalstep) then
                   x(ix)=x(ix)+.sx(ix)
                   mymotor(ix).stepping(.sx(ix))
                   .cx(ix)=.cx(ix)-.totalstep
                end if 
            next
            '' next speed
            if i<=.rampup then
                ''if (.ac1>0 and f<.fn) or ( .ac1<0 and f>.fn) then 
                f=f+.ac1*dl
            elseif i>=.totalstep-.rampdown then
                ''if (.ac2>0 and f<=.fe) or ( .ac2<0 and f>=.fe) then 
                f=f+.ac2*dl
            else 
                f=.fn
            end if

        next
        print f
        .status=0
    end with
end sub
function startmove() as integer
    if (head<>tail) then
        tail=nextbuff(tail)
        currmove=@move(tail)
        gogo(tail)
        startmove=1
    else
        startmove=0
    end if
end function
sub waitbuffer()        
    if head<>tail then 
        prepareramp(head)
        do
        loop until startmove()=0
    end if
end sub
print "Simple Motion Control with Acceleration, Jerk, and lookahead planner"
print "By ryannining@gmail.com"
function degtorad(d as double) as double
    degtorad=d*22/(7*180)
end function

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

accel=10
addmove(50,100,100,0)
addmove(50,240,200,0)
addmove(50,440,100,0)
addmove(50,300,100,0)
addmove(50,50,200,0)

dim as integer i
dim as double di
for i=0 to 20
    dim r as double
    di=(i and 31)/31.0
    r=100
    addmove(20,-cos(degtorad(di*350))*r*3+300,-sin(degtorad(di*350))*r+100,0)    
    ''addmove(100,-cos(degtorad(i*180+2))*20*3+300,i*4+20,0)    
    if bufflen()>4 then startmove()
next
'/

addmove(100,300,100,0)
addmove(100,500,10,0)
line (0,400)-(600,400),1
line (0,400-50*2)-(600,400-50*2),1
line (0,400-100*2)-(600,400-100*2),1
line (0,450)-(600,450),2

waitbuffer()
print "Time:",tick



sleep
