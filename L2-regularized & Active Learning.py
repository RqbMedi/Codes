import sys
import numpy as np
import copy

lamb, sigma2, X, y, x = sys.argv[1:]
lamb=float(lamb)
X = np.genfromtxt (X, delimiter=",")
y = np.genfromtxt (y, delimiter=",")
x = np.genfromtxt (x, delimiter=",")
I=np.identity(len(X[0]))
XT=np.transpose(X)			
A=np.dot(lamb,I)+np.dot(XT,X)
B=np.linalg.inv(A)
C=np.dot(XT,y)
wRR=np.dot(B,C)

with open('wRR_'+sys.argv[1]+'.csv','w') as o:
 for i in range(0,len(wRR)):
   o.write("%s\n" % wRR[i])
 
sigma2=float(sigma2)
A1=np.dot(XT,X)
A2=np.dot(1/sigma2,A1)
A3=np.dot(lamb,I)+A2
Eprior=np.linalg.inv(A3)
elements=[]
sigmaelements=[]
Ep=Eprior
sigma0=0
c=0
ListIndex=[]


for j in range(0,10):
  for i in range(0,len(x)):
    sigma2x0=sigma2+np.dot(np.transpose(x[i]),np.dot(Ep,x[i]))
    if sigma2x0 > sigma0 and i not in ListIndex:
      c=i
    sigma0=sigma2x0 
  ListIndex.append(c)
  d=x[c]
  E0=np.transpose(x[c])
  E1=np.dot(d,E0)
  E2=np.dot(1/sigma2,E1)
  E3=np.linalg.inv(Eprior)
  E4=E3+E2
  Ep=np.linalg.inv(E4)

print(ListIndex)
with open('active_'+sys.argv[1]+'_'+sys.argv[2]+'.csv','w') as o:
 for i in range(0,len(ListIndex)-1): 
   o.write("%s," % str(ListIndex[i]+1))
 o.write("%s" % str(ListIndex[len(ListIndex)-1]+1))
 
 
  
 
 
 
