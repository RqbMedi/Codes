import sys
import math
import copy

grid=[]
row=[]
for n in range(0,len(sys.argv[1])):
  a=int(sys.argv[1][n])
  row.append(a)
  if len(row)==9:
    row1=copy.deepcopy(row)
    grid.append(row1)
    row=[]

def Lineabsent(k,grid,i):
    for j in range(0,9):
        if grid[i][j]==k:
            return False
    return True

def Colabsent(k,grid,j):
    for i in range(0,9):
        if grid[i][j]==k:
            return False
    return True

def Groupabsent(k,grid,i,j):
    h=copy.deepcopy(i)
    f=copy.deepcopy(j)
    h=h-h%3
    f=f-f%3
    for p in range(h,h+3):
        for t in range(f,f+3):
            if grid[p][t]==k:
                return False
    return True

def Good(grid,pos):
    if(pos==9*9):
        return True

    i=int(pos/9)
    j=pos%9

    if grid[i][j]!=0:
        return Good(grid,pos+1)
    for k in range(1,10):
        if Lineabsent(k,grid,i) and Colabsent(k,grid,j) and Groupabsent(k,grid,i,j):
            grid[i][j]=k
            if Good(grid,pos+1):
                return True
    grid[i][j]=0
    return False

Good(grid,0)

result=''

for i in range(0,9):
  for j in range(0,9):
    result=result+str(grid[i][j])

with open('output.txt','w') as o:
        o.write(str(result) + '\n')


   

    



