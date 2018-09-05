from scipy import sparse
from scipy.stats import multivariate_normal
import numpy as np
import sys
import math

X = sys.argv[1]
l, c, v = np.loadtxt(X,delimiter=',').T

##l, c, v = np.loadtxt('ratings.csv',delimiter=',').T
m = sparse.coo_matrix((v, (l-1, c-1)), shape=(l.max(), c.max()))
b=m.toarray()

##print(b.shape[1])

y=np.zeros(5)
z=2*np.identity(5)

vj=[]
for i in range(b.shape[1]):
 v1 = np.random.multivariate_normal(y, z).T
 vj.append(v1)

omega_ui=[]
omega_vj=[]

w=np.transpose(b)

for i in range(b.shape[0]):
  idxs_ui=np.nonzero(b[i])
  omega_ui.append(idxs_ui[0])

for j in range(b.shape[1]):
  idxs_vj=np.nonzero(w[j])
  omega_vj.append(idxs_vj[0])
  
q=np.nonzero(b)


ui=[]
for i in range(b.shape[0]):
 u1 = np.random.multivariate_normal(y, z).T
 ui.append(u1)

L=[]

for h in range(50):

 for i in range(b.shape[0]):
  u1=0.2*np.identity(5)
  u2=sum(np.outer(vj[j],np.transpose(vj[j])) for j in omega_ui[i])
  u3=u1+u2
  u4=np.linalg.inv(u3)
  u5=sum(b[i][j]*vj[j] for j in omega_ui[i])
  u6=np.dot(u4,u5)
  ui[i]=u6


 for j in range(b.shape[1]):
  v1=0.2*np.identity(5)
  v2=sum(np.outer(ui[i],np.transpose(ui[i])) for i in omega_vj[j])
  v3=v1+v2
  v4=np.linalg.inv(v3)
  v5=sum(b[i][j]*ui[i] for i in omega_vj[j])
  v6=np.dot(v4,v5)
  vj[j]=v6
  
 l1=-5*sum(math.pow(b[q[0][t]][q[1][t]]-np.dot(np.transpose(ui[q[0][t]]),vj[q[1][t]]),2) for t in range(len(q[1])))
 l2=-sum(np.dot(np.transpose(ui[i]),ui[i]) for i in range(len(ui)))
 l3=-sum(np.dot(np.transpose(vj[j]),vj[j]) for j in range(len(vj)))
 l4=l1+l2+l3
 L.append(l4)
 
 if h==9:
      with open('U-'+str(10)+'.csv','w') as o:
        for i in range(0,len(ui)):
          o.write("%s\n" % str(ui[i]))
      with open('V-'+str(10)+'.csv','w') as o:
        for i in range(0,len(vj)):
          o.write("%s\n" % str(vj[i]))
 if h==24:
      with open('U-'+str(25)+'.csv','w') as o:
        for i in range(0,len(ui)):
          o.write("%s\n" % str(ui[i]))
      with open('V-'+str(25)+'.csv','w') as o:
        for i in range(0,len(vj)):
          o.write("%s\n" % str(vj[i]))
 if h==49:
      with open('U-'+str(50)+'.csv','w') as o:
        for i in range(0,len(ui)):
          o.write("%s\n" % str(ui[i]))
      with open('V-'+str(50)+'.csv','w') as o:
        for i in range(0,len(vj)):
          o.write("%s\n" % str(vj[i]))
 
with open('objective.csv','w') as o:
   for i in range(0,len(L)):
       o.write("%s\n" % str(L[i])) 

