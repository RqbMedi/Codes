import sys
import numpy as np
import copy
import math
import csv
 
X1, y1, x1 = sys.argv[1:]
X = np.genfromtxt (X1,delimiter=",")
y = np.genfromtxt (y1,delimiter=",")
x = np.genfromtxt (x1,delimiter=",")


##y = np.genfromtxt ("Y_train.csv", delimiter=",")
##x = np.genfromtxt ("X.csv",delimiter=",")
##X = np.genfromtxt ("X _train.csv",delimiter=",")
##print(X)

pi=[]
mu=[]
cov=[]
ny=0
idxs=0
n=len(y)


for j in range(0,10):
  covy=0
  covidxs=[]
  piy=0
  muy=0
  ny=np.count_nonzero(y==j)
  idxs=np.nonzero(y==j)
  piy=len(idxs[0])*(1.0/n)
  muy=(1.0/ny)*sum(X[i] for i in idxs[0])
  s=(len(X[0]),len(X[0]))
  covidxs=[]
  for i in idxs[0]:
   covyi=np.outer(X[i]-muy,np.transpose(X[i]-muy))
   covidxs.append(covyi)
  covy=(1.0/(ny-1))*sum(t for t in covidxs)
  pi.append(piy)
  mu.append(muy)
  cov.append(covy)
  
probs=[]

for a in x:
 prob=[]
 for b in range(10):
        p1 = pi[b] * math.pow(np.linalg.norm(cov[b]), -0.5)
        p2 = a-mu[b]
        p3 = np.transpose(p2)
        p4 = np.linalg.inv(cov[b])
        p5 = np.dot(p4,p2)
        p6 = np.dot(p3,p5)
        p7 = np.dot(-0.5,p6)
        p8 = math.exp(p7)
        p9 = p1*p8
        prob.append(p9)
 probs.append(prob)
 

with open('probs_test.csv','w') as o:
  for i in range(0,len(probs)):
     c=",".join([str(p) for p in probs[i]]) 
     o.write("%s\n" % c)


