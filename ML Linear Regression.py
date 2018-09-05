import csv
import sys
import random
import copy
import statistics

##f = open(sys.argv[1], 'rt')
##try:
##    reader = csv.reader(f)
##    for row in reader:
##        print row
##finally:
##    f.close()


def sign(x):
    if x>=0:
        return 1
    else:
        return -1

a=[]
d=[]
l=[]
n=[]

h = open('input2.csv')
csv_h = csv.reader(h)
for row in csv_h:
 for x in row:
    d.append(float(x))
 c=d.pop()
 d.append(1)
 d.append(c)
 a.append(d)
 d=[]

for i in range(0,len(a)):
   l.append(a[i][0])
   n.append(a[i][1])

lmean=copy.deepcopy(l)
lstd=copy.deepcopy(l)
meanl=statistics.mean(l)
stdl=statistics.stdev(l)

nmean=copy.deepcopy(n)
nstd=copy.deepcopy(n)
meann=statistics.mean(n)
stdn=statistics.stdev(n)

for i in range(0,len(a)):
   a[i][0]=(a[i][0]-meanl)/stdl
   a[i][1]=(a[i][1]-meann)/stdn

def f(b0,b1,x1,b2,x2):
    z=0
    z=b0+b1*x1+b2*x2
    return z
converge=[]

##with alpha=0.001
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.001*(1/len(a))*o
    b1=b1-0.001*(1/len(a))*w
    b2=b2-0.001*(1/len(a))*s
    p=p+1
k=[]
k.append(0.001)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m1=copy.deepcopy(k)
converge.append(m1)
    

##with alpha=0.005
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.005*(1/len(a))*o
    b1=b1-0.005*(1/len(a))*w
    b2=b2-0.005*(1/len(a))*s
    p=p+1
k=[]
k.append(0.005)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m2=copy.deepcopy(k)
converge.append(m2)

##with alpha=0.01
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.01*(1/len(a))*o
    b1=b1-0.01*(1/len(a))*w
    b2=b2-0.01*(1/len(a))*s
    p=p+1
k=[]
k.append(0.01)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m3=copy.deepcopy(k)
converge.append(m3)

##with alpha=0.05
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.05*(1/len(a))*o
    b1=b1-0.05*(1/len(a))*w
    b2=b2-0.05*(1/len(a))*s
    p=p+1
k=[]
k.append(0.05)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m4=copy.deepcopy(k)
converge.append(m4)

##with alpha=0.1
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.1*(1/len(a))*o
    b1=b1-0.1*(1/len(a))*w
    b2=b2-0.1*(1/len(a))*s
    p=p+1
k=[]
k.append(0.1)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m5=copy.deepcopy(k)
converge.append(m5)

##with alpha=0.5
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-0.5*(1/len(a))*o
    b1=b1-0.5*(1/len(a))*w
    b2=b2-0.5*(1/len(a))*s
    p=p+1
k=[]
k.append(0.5)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m6=copy.deepcopy(k)
converge.append(m6)

##with alpha=1
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-1*(1/len(a))*o
    b1=b1-1*(1/len(a))*w
    b2=b2-1*(1/len(a))*s
    p=p+1
k=[]
k.append(1)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m7=copy.deepcopy(k)
converge.append(m7)

##with alpha=5
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-5*(1/len(a))*o
    b1=b1-5*(1/len(a))*w
    b2=b2-5*(1/len(a))*s
    p=p+1
k=[]
k.append(5)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m8=copy.deepcopy(k)
converge.append(m8)

##with alpha=10
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-10*(1/len(a))*o
    b1=b1-10*(1/len(a))*w
    b2=b2-10*(1/len(a))*s
    p=p+1
k=[]
k.append(10)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m9=copy.deepcopy(k)
converge.append(m9)

##with alpha=8
o=0
w=0
s=0
p=0
b0=0
b1=0
b2=0
while p!=100:
    for i in range(0,len(a)):
       o=o+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])
       w=w+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][0]
       s=s+(f(b0,b1,a[i][0],b2,a[i][1])-a[i][3])*a[i][1]
    b0=b0-8*(1/len(a))*o
    b1=b1-8*(1/len(a))*w
    b2=b2-8*(1/len(a))*s
    p=p+1
k=[]
k.append(8)
k.append(100)
k.append(b0)
k.append(b1)
k.append(b2)
m10=copy.deepcopy(k)
converge.append(m10)


with open(sys.argv[2],'w') as o:
 for i in range(0,len(converge)):
   o.write("%s,%s,%s,%s,%s\n" % (converge[i][0],converge[i][1],converge[i][2],converge[i][3],converge[i][4]))
   
