function ramplen(v0 as double,v1 as double,a as double,st as double) as double
    dim t as double
    t=abs(v1-v0)/a
    ramplen=abs((v0*t+0.5*a*t^2)*st)
end function
function speedat(v0 as double,a as double,st as double) as double
    speedat=sqr(a*2*st+v0*v0)
end function
function accelat(v0 as double,v1  as double,st as double) as double
    ''v1=sqr(a*2*st+v0*v0)
    ''v1*v1-v0*v0
    accelat=(v1*v1-v0*v0)/(2*st)
end function

dim as double r,a
r=ramplen(0,100,100,1)
print r
print speedat(88,-19,1170.0/45)
print "----"
a=accelat(88,68,1170.0/45)
print a
print speedat(88,a,1170.0/45)
sleep
