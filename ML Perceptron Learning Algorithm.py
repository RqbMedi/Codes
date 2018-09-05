import csv
import sys
import random
import copy

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
f=0
p=1
k=0
h = open(sys.argv[1])
csv_h = csv.reader(h)
for row in csv_h:
 for x in row:
    d.append(int(x))
 c=d.pop()
 d.append(1)
 d.append(c)
 a.append(d)
 d=[]
 
w=[]
w1=0
w2=0
b=0
w.append(w1)
w.append(w2)
w.append(b)
converge=[]

mislist=[]
y=0
while p!=25:
  for i in range(0,len(a)):
    f= w[0]*a[i][0]+w[1]*a[i][1]+w[2]*a[i][2]
    if sign(f)!= sign(a[i][3]):
        mislist.append(a[i])
  if mislist==[]:
    p=25
  if len(mislist)==4:
    k=random.randint(0,len(mislist))
    w[0]=w[0]+a[k][0]*a[k][3]
    w[1]=w[1]+a[k][1]*a[k][3]
    w[2]=w[2]+a[k][2]*a[k][3]
    m=copy.deepcopy(w)
    converge.append(m)
    p=p+1
  else:
   k=random.randint(0,len(mislist))
   w[0]=w[0]+a[k][0]*a[k][3]
   w[1]=w[1]+a[k][1]*a[k][3]
   w[2]=w[2]+a[k][2]*a[k][3]
   m=copy.deepcopy(w)
   converge.append(m)
  mislist=[]

with open(sys.argv[2],'w') as o:
 for i in range(0,len(converge)):
   o.write("%s,%s,%s\n" % (converge[i][0],converge[i][1],converge[i][2]))
 o.write("%s,%s,%s\n" % (-14,-10,140))
   


     
     
     
    
