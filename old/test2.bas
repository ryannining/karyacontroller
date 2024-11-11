screen 12
dim as double t,v0,v1,a,v,ramp,total
dim as integer i
v0=40
v1=100
a=7

t=(v1-v0)/a


ramp=v0*t+0.5*a*t^2
total=100
print ramp,total
/'
if ramp>total then
    do
        v1=v0+0.8*(v1-v0)*sqr(total/ramp)
        t=(v1-v0)/a
        ramp=v0*t+0.5*a*t^2
        print ramp,total
    loop until ramp<=total+1
end if
function speedat(v0 as double,a as double,st as double) as double
    speedat=sqr(a*2*st+v0^2)
end function
'/
if ramp>total then
    /'a=(v1^2 -v0^2)/2*d
    ' a2d=v1^2-v0^2
    'v1^2=a2d+v0^2
    v1=sqr(a2d-v0^2)
    '/
        v1=sqr(a*2*total+v0^2)
        print v1
        t=(v1-v0)/a
        ramp=v0*t+0.5*a*t^2
        print ramp,total
end if

sleep
