screen 12
dim as double stepm,ramp,a,v,v0,v1,tick,dl,pdl
function ramplen(v0 as double,v1 as double,a as double,st as double) as double
    dim t as double
    t=(v1-v0)/a
    ramplen=(v0*t+0.5*a*t^2)*st
end function

stepm=100
a=60
v0=1
v1=60
line (100,200-v1)-(600,200-v1),1
line (100,200)-(600,200),1

dim as integer i
''s=1/2at^2
''t^2=1/s2a
''
dl=abs(v1-v0)/(a)
ramp=ramplen(v0,v1,a,stepm)
tick=0
v=v0
pdl=1
for i=1 to 300*stepm 
    if v>0 then 
        dl=1/(v*stepm)
    else 
        dl=1/stepm
    end if
    tick=tick+dl
    pset (100+i/stepm,200-v),12
    pset (100+tick*40,400-v),12
    if i<ramp then v=v+a*dl
    pdl=dl
next
color 12
print "Time:",tick
a=a/stepm/10
ramp=(v1-v0)/(a)
tick=0
v=v0
for i=1 to 300*stepm
    if v>0 then 
        dl=1/(v*stepm)
    else 
        dl=1/stepm
    end if
    tick=tick+dl
    pset (100+i/stepm,200-v),10
    pset (100+tick*40,400-v),10
    if i<ramp then v=v+a
next
color 10
print "Time:",tick
sleep