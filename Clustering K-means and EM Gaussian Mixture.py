from __future__ import division
from scipy.stats import multivariate_normal
import numpy as np
import sys
import copy

X = sys.argv[1]
X = np.genfromtxt(X,delimiter=",")

##X = np.genfromtxt ("X.csv",delimiter=",")
centroids = X[np.random.choice(np.arange(len(X)), 5), :]

for i in range(10):
   # Cluster Assignment step
   C = np.array([np.argmin([np.dot(x_i-y_k, x_i-y_k) for y_k in centroids]) for x_i in X])
   # Move centroids step
   centroids = [X[C == k].mean(axis = 0) for k in range(5)]
   with open('centroids-'+str(i+1)+'.csv','w') as o:
     for j in range(0,len(centroids)):
       c=",".join([str(p) for p in centroids[j]])
       o.write("%s\n" % c)

       
pi=np.zeros(5)
for i in range(5):
     pi[i]=1/5

means = X[np.random.choice(np.arange(len(X)), 5), :]

sigmas=[]
for i in range(5):
  sigma=np.identity(len(X[0]))
  sigmas.append(sigma)
 

for t in range(1,11):
 phi=[]
 n_kall=[]
 for i in range(len(X)):
     phi_i=[]
     for k in range(5):
       phi_k=np.dot(pi[k],multivariate_normal.pdf(X[i],means[k],sigmas[k]))/sum(np.dot(pi[j],multivariate_normal.pdf(X[i],means[j],sigmas[j])) for j in range(5))
       phi_i.append(phi_k)
     phi.append(phi_i)
 

 for k in range(5):
   n_k = sum(phi[i][k] for i in range(len(X)))
   n_kall.append(n_k)

   pi[k]=n_kall[k]/len(X)

   means[k]=sum(np.dot(phi[i][k],X[i]) for i in range(len(X)))/n_kall[k]

   sigmas[k]=sum(phi[i][k]*np.outer(X[i]-means[k],np.transpose(X[i]-means[k])) for i in range(len(X)))/n_kall[k]

 with open('pi-'+str(t)+'.csv','w') as o:
     for j in range(0,len(pi)):
       o.write("%s\n" % str(pi[j]))   

 with open('mu-'+str(t)+'.csv','w') as o:
     for j in range(0,len(pi)):
       c=",".join([str(p) for p in means[j]])
       o.write("%s\n" % c)  

 ##for j in range(5):    
 ##  with open('Sigma-'+str(j+1)+'-'+str(t)+'.csv','w') as o:
 ##    for i in range(len(sigmas[j])): 
 ##       c=",".join([str(p) for p in sigmas[j][i]])
 ##       o.write("%s\n" % c)  
 
 with open('Sigma-'+str(1)+'-'+str(t)+'.csv','w') as o:
     for i in range(len(sigmas[0])): 
        c=",".join([str(p) for p in sigmas[0][i]])
        o.write("%s\n" % c)  
      
 with open('Sigma-'+str(2)+'-'+str(t)+'.csv','w') as o:
     for i in range(len(sigmas[1])): 
        c=",".join([str(p) for p in sigmas[1][i]])
        o.write("%s\n" % c)
      
 with open('Sigma-'+str(3)+'-'+str(t)+'.csv','w') as o:
     for i in range(len(sigmas[2])): 
        c=",".join([str(p) for p in sigmas[2][i]])
        o.write("%s\n" % c)
      
 with open('Sigma-'+str(4)+'-'+str(t)+'.csv','w') as o:
     for i in range(len(sigmas[3])): 
        c=",".join([str(p) for p in sigmas[3][i]])
        o.write("%s\n" % c)
      
 with open('Sigma-'+str(5)+'-'+str(t)+'.csv','w') as o:
     for i in range(len(sigmas[4])): 
        c=",".join([str(p) for p in sigmas[4][i]])
        o.write("%s\n" % c)      

