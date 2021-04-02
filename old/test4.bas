screen 12
dim as double f,tick,c1,i,ru,rd,c0,a,k,ni
ru=50
rd=50
c0=10
tick=0
k=0
for i=1 to 300-1
    ni=(i-300)
    if i<ru then c1=c0-(2*c0/(4*i+1+k*i))
    if i>300-rd then c1=c0-(2*c0/(4*ni+1+k*ni))
    tick+=c1
    f=1/c1
    ''print 20/f
    pset (tick/13,200-200*f)
    c0=c1
next
sleep
