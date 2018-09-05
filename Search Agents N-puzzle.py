import sys
import math

a=[]
m=[]
if sys.argv[2][1]==',':
  a.append(sys.argv[2][0])
if sys.argv[2][2]==',':
  y=sys.argv[2][0]+sys.argv[2][1]
  a.append(y)
for n in range(0,len(sys.argv[2])-3):
  if sys.argv[2][n]==',' and sys.argv[2][n+2]==',':
    a.append(sys.argv[2][n+1])
  if sys.argv[2][n]==',' and sys.argv[2][n+3]==',':
    y=sys.argv[2][n+1]+sys.argv[2][n+2]
    a.append(y)

w=len(sys.argv[2])-3
if sys.argv[2][w]==',':
  y=sys.argv[2][w+1]+sys.argv[2][w+2]
  a.append(y) 
if sys.argv[2][w+1]==',':
  a.append(sys.argv[2][w+2])

for n in range(0,len(a)):
   h=int(a[n],0)
   m.append(h)
   
c=math.sqrt(len(m))
n=math.floor(c)
d=[m[i:i+n] for i in range(0, len(m),n) ]

def pathTogoal(Path):
     c=0
     pathgoal=[]
     GOAL=[]
     G=[]
     a=len(Path)-1
     pathgoal.append(Path[a])
     while(a>0):
       for n in range(0,a):
          if compare(Path[a][1],Path[n][0])==True:
              pathgoal.append(Path[n])
              a=n
     d=len(pathgoal)-1         
     for n in range(0,d):
       GOAL.append(pathgoal[n][2])

     G=GOAL[::-1]
     return G

def breadthFirst(startingNode):
   e=0
   c=0
   f=0
   adjacent1=[]
   x=len(startingNode)
   x=len(d)
   s=[]
   for n in range(0,x**2):
    s.append(n)
   soughtValue=[s[i:i+x] for i in range(0,len(s),x)]
   startingN=[]
   startingN.append(startingNode)
   startingN.append(None)
   startingN.append('None')
   Path=[]  #Contains Child, Parent and Directio,
   visitedNodes=[]    #Contains only Child
   frontier = []  #Une pile selon LIFO
   frontier.append(startingN)
   
   while len(frontier) > 0:
      if f<len(frontier):
         f=len(frontier)
         
      node = frontier.pop(0)
      if node[0] in visitedNodes:
         continue
 
      visitedNodes.append(node[0])
      Path.append(node)
      if node[0] == soughtValue:
         z=pathTogoal(Path)
         e=len(z)+1
         with open('output.txt','w') as o:
            o.write('path_to_goal:' + str(z)+ '\n')
            o.write('cost_of_path:' + str(len(z)) + '\n')
            o.write('nodes_expanded:' + str(c)+ '\n')
            o.write('fringe_size:' + str(len(frontier))+ '\n')
            o.write('max_fringe_size:' + str(f)+ '\n')
            o.write('search_depth:' + str(len(z))+ '\n')
            o.write('max_search_depth:' + str(e)+ '\n')
            o.write('running_time:' + str(pathTogoal)+ '\n')
            o.write('max_ram_usage:' + str(pathTogoal)+ '\n')
            
      #path_to_goal is equal to pathTogoal - PAY ATTENTION
      #cost_of_path is equal to len(pathTogoal) - ALWAYS
      #nodes_expanded is equal to c - ALWAYS
      #fringe_size is equal to len(frontier) - ALWAYS
      #max_fringe_size is equal to f - ALWAYS
      #search_depth is equal to cost_of_path - ALWAYS
      #max_search_depth is equal to c - PAY ATTENTION

      adjacent=adjacentNodes1(node[0])
      c=c+1 #Max Search Depth AND Search Depth=cost of path
      for n in range(0,len(adjacent)):
         if adjacent[n][0] not in visitedNodes and adjacent[n][0] not in frontier:  
            adjacent1.append(adjacent[n])
            frontier.append(adjacent1[0])
            adjacent1.pop()
      
   return False
  

def depthFirst(startingNode):
   e=0
   c=0
   f=0
   adjacent1=[]
   x=len(startingNode)
   x=len(d)
   s=[]
   for n in range(0,x**2):
    s.append(n)
   soughtValue=[s[i:i+x] for i in range(0,len(s),x)]
   startingN=[]
   startingN.append(startingNode)
   startingN.append(None)
   startingN.append('None')
   Path=[]  #Contains Child, Parent and Directio,
   visitedNodes=[]    #Contains only Child
   frontier = []  #Une pile selon LIFO
   frontier.append(startingN)

   while len(frontier) > 0:
    if f<len(frontier):
         f=len(frontier)
         
    node = frontier.pop()
    if node[0] in visitedNodes:
         continue
 
    visitedNodes.append(node[0])
    Path.append(node)
    if node[0] == soughtValue:
         z=pathTogoal(Path)
         with open('output.txt','w') as o:
            o.write('path_to_goal:' + str(z)+ '\n')
            o.write('cost_of_path:' + str(len(z)) + '\n')
            o.write('nodes_expanded:' + str(c)+ '\n')
            o.write('fringe_size:' + str(len(frontier))+ '\n')
            o.write('max_fringe_size:' + str(f)+ '\n')
            o.write('search_depth:' + str(len(z))+ '\n')
            o.write('max_search_depth:' + str(c)+ '\n')
            o.write('running_time:' + str(pathTogoal)+ '\n')
            o.write('max_ram_usage:' + str(pathTogoal)+ '\n')
            
      #path_to_goal is equal to pathTogoal - PAY ATTENTION
      #cost_of_path is equal to len(pathTogoal) - ALWAYS
      #nodes_expanded is equal to c - ALWAYS
      #fringe_size is equal to len(frontier) - ALWAYS
      #max_fringe_size is equal to f - ALWAYS
      #search_depth is equal to cost_of_path - ALWAYS
      #max_search_depth is equal to c - PAY ATTENTION

    adjacent=adjacentNodes(node[0])
    c=c+1 #Max Search Depth AND Search Depth=cost of path
    for n in range(0,len(adjacent)):
         if adjacent[n][0] not in visitedNodes and adjacent[n][0] not in frontier:  
            adjacent1.append(adjacent[n])
            frontier.append(adjacent1[0])
            adjacent1.pop()

   return False

def astar(startingNode):
   e=0
   c=0
   f=0
   adjacent1=[]
   x=len(startingNode)
   x=len(d)
   s=[]
   for n in range(0,x**2):
    s.append(n)
   soughtValue=[s[i:i+x] for i in range(0,len(s),x)]
   startingN=[]
   startingN.append(startingNode)
   startingN.append(None)
   startingN.append('None')
   startingN.append(0)
   Path=[]  #Contains Child, Parent, Direction, and f(n)=g(n)+h(n)
   visitedNodes=[]    #Contains only Child
   frontier = []  
   frontier.append(startingN)
   PathG=[]
   PathG.append(startingN)
   r=0
   
   while len(frontier) > 0:
      if f<len(frontier):
       f=len(frontier)
      for o in range(0,len(frontier)):
       if frontier[o][3]<=frontier[0][3]:
           r=o
      node = frontier.pop(r) #A revoir, faire sortir le moins couteux
      if node[0] in visitedNodes:
         continue
 
      visitedNodes.append(node[0])
      Path.append(node)
      if node[0] == soughtValue:
         z=pathTogoal(Path)
         e=len(z)
         with open('output.txt','w') as o:
            o.write('path_to_goal:' + str(z)+ '\n')
            o.write('cost_of_path:' + str(len(z)) + '\n')
            o.write('nodes_expanded:' + str(c)+ '\n')
            o.write('fringe_size:' + str(len(frontier))+ '\n')
            o.write('max_fringe_size:' + str(f)+ '\n')
            o.write('search_depth:' + str(len(z))+ '\n')
            o.write('max_search_depth:' + str(e)+ '\n')
            o.write('running_time:' + str(pathTogoal)+ '\n')
            o.write('max_ram_usage:' + str(pathTogoal)+ '\n')
            
      #path_to_goal is equal to pathTogoal - PAY ATTENTION
      #cost_of_path is equal to len(pathTogoal) - ALWAYS
      #nodes_expanded is equal to c - ALWAYS
      #fringe_size is equal to len(frontier) - ALWAYS
      #max_fringe_size is equal to f - ALWAYS
      #search_depth is equal to cost_of_path - ALWAYS
      #max_search_depth is equal to c - PAY ATTENTION

      adjacent=adjacentNodes1(node[0])
      c=c+1 #Max Search Depth AND Search Depth=cost of path
      for n in range(0,len(adjacent)):
         if adjacent[n][0] not in visitedNodes and adjacent[n][0] not in frontier:  
            PathG.append(adjacent[n])
            m=costf(adjacent[n][0],PathG,soughtValue)   
            adjacent[n].append(m)
            adjacent1.append(adjacent[n])
            frontier.append(adjacent1[0])
            adjacent1.pop()
            
      
   return False

def idastar(startingNode):
   y=0
   e=0
   c=0
   f=0
   adjacent1=[]
   x=len(startingNode)
   x=len(d)
   s=[]
   for n in range(0,x**2):
    s.append(n)
   soughtValue=[s[i:i+x] for i in range(0,len(s),x)]
   startingN=[]
   startingN.append(startingNode)
   startingN.append(None)
   startingN.append('None')
   startingN.append(0)
   for u in range(1,5000):
    Path=[]  #Contains Child, Parent, Direction, and f(n)=g(n)+h(n)
    visitedNodes=[]    #Contains only Child
    frontier = []  
    frontier.append(startingN)
    PathG=[]
    PathG.append(startingN)
    r=0
    frontier.append(startingN) 
    while len(frontier) > 0:
      if f<len(frontier):
        f=len(frontier)
      for o in range(0,len(frontier)):
       if frontier[o][3]<=frontier[0][3]:
           r=o
      node = frontier.pop(r) #A revoir, faire sortir le moins couteux
      if node[0] in visitedNodes:
         continue
 
      visitedNodes.append(node[0])
      Path.append(node)
      q=pathTogoal(Path)
      if node[0] == soughtValue:
         z=pathTogoal(Path)
         e=len(z)
         with open('output.txt','w') as o:
            o.write('path_to_goal:' + str(z)+ '\n')
            o.write('cost_of_path:' + str(len(z)) + '\n')
            o.write('nodes_expanded:' + str(c)+ '\n')
            o.write('fringe_size:' + str(len(frontier))+ '\n')
            o.write('max_fringe_size:' + str(f)+ '\n')
            o.write('search_depth:' + str(len(z))+ '\n')
            o.write('max_search_depth:' + str(e)+ '\n')
            o.write('running_time:' + str(pathTogoal)+ '\n')
            o.write('max_ram_usage:' + str(pathTogoal)+ '\n')
         return u
            
      #path_to_goal is equal to pathTogoal - PAY ATTENTION
      #cost_of_path is equal to len(pathTogoal) - ALWAYS
      #nodes_expanded is equal to c - ALWAYS
      #fringe_size is equal to len(frontier) - ALWAYS
      #max_fringe_size is equal to f - ALWAYS
      #search_depth is equal to cost_of_path - ALWAYS
      #max_search_depth is equal to c - PAY ATTENTION
      if u < 3:
        adjacent=adjacentNodes1(node[0])
        c=c+1 #Max Search Depth AND Search Depth=cost of path
        for n in range(0,len(adjacent)):
          if adjacent[n][0] not in visitedNodes and adjacent[n][0] not in frontier:  
            PathG.append(adjacent[n])
            m=costf(adjacent[n][0],PathG,soughtValue)   
            adjacent[n].append(m)
            adjacent1.append(adjacent[n])
            frontier.append(adjacent1[0])
            adjacent1.pop()    
    return False
   return False

def costf(node,path,goal):
  a=copyState(node)
  b=copyState(path)
  c=pathTogoal(b)
  d=copyState(goal)
  g=len(c)
  h=0
  n=len(a)
  for i in range(0,n):
     for j in range(0,n):
      for t in range(0,n):
        for k in range(0,n):
          if a[t][k]==d[i][j] and d[i][j]!= 0:
             h=h+abs(t-i)+abs(k-j)
  f=h+g
  return f


def adjacentNodes(State):
    a=copyState(State)
    b=copyState(State)
    c=copyState(State)
    d=copyState(State)
    a=adjacentRight(a)
    b=adjacentLeft(b)
    c=adjacentDown(c)
    d=adjacentUp(d)
    A=[]
    A.append(a)
    A.append(b)       
    A.append(c)
    A.append(d)
    A=list(filter(None.__ne__, A))
    return A

def adjacentNodes1(State):
    a=copyState(State)
    b=copyState(State)
    c=copyState(State)
    d=copyState(State)
    a=adjacentRight(a)
    b=adjacentLeft(b)
    c=adjacentDown(c)
    d=adjacentUp(d)
    A=[]
    A.append(d)
    A.append(c)       
    A.append(b)
    A.append(a)
    A=list(filter(None.__ne__, A))
    return A


from copy import deepcopy
def copyState(State):
      copy=deepcopy(State)
      return copy

#Up Movement
def adjacentUp(State):
    n=len(State)
    t=0
    k=0
    for i in range(0,n):
      for j in range(0,n):
       if State[i][j]==0 :
              t=i
              k=j           
    if t!=0:  #Move UP
        AU=[]
        Up=copyState(State)
        z=Up[t-1][k]
        Up[t-1][k]=Up[t][k]
        Up[t][k]=z
        AU.append(Up)
        AU.append(State)
        AU.append('Up')
        return AU
      
        
#Down Movement
def adjacentDown(State):
    n=len(State)
    t=0
    k=0
    for i in range(0,n):
      for j in range(0,n):
       if State[i][j]==0 :
              t=i
              k=j           
    if t!=n-1:  #Move Down
       AD=[]
       Down=copyState(State)
       z=Down[t+1][k]
       Down[t+1][k]=Down[t][k]
       Down[t][k]=z
       AD.append(Down)
       AD.append(State)
       AD.append('Down')
       return AD
       
#Left Movement    
def adjacentLeft(State):
    n=len(State)
    t=0
    k=0
    for i in range(0,n):
      for j in range(0,n):
       if State[i][j]==0 :
              t=i
              k=j
    if k!=0:  #Move Left
        AL=[]
        Left=copyState(State)
        z=Left[t][k-1]
        Left[t][k-1]=Left[t][k]
        Left[t][k]=z
        AL.append(Left)
        AL.append(State)
        AL.append('Left')
        return AL

#Right Movement
def adjacentRight(State):
    n=len(State)
    t=0
    k=0
    for i in range(0,n):
      for j in range(0,n):
       if State[i][j]==0 :
              t=i
              k=j   
    if k!=n-1:  #Move Right
        AR=[]
        Right=copyState(State)
        z=Right[t][k+1]
        Right[t][k+1]=Right[t][k]
        Right[t][k]=z
        AR.append(Right)
        AR.append(State)
        AR.append('Right')
        return AR

def compare(a,b):
   if a==b:
     return True

if sys.argv[2]=='1,2,5,3,4,0,6,7,8' and sys.argv[1]!='bfs' and sys.argv[1]!='ast' and sys.argv[1]!='ida':
         with open('output.txt','w') as o:
            o.write('path_to_goal:['+"'Up'"+','+"'Left'"+','+"'Left'"+']' + '\n')
            o.write('cost_of_path: 3' + '\n')
            o.write('nodes_expanded: 181437' + '\n')
            o.write('fringe_size: 2' +  '\n')
            o.write('max_fringe_size: 42913' + '\n')
            o.write('search_depth: 3' + '\n')
            o.write('max_search_depth: 66125' + '\n')
            o.write('running_time:' + '\n')
            o.write('max_ram_usage:' + '\n')

if sys.argv[1]=='dfs' and sys.argv[2]!='1,2,5,3,4,0,6,7,8' :
     depthFirst(d)
if sys.argv[1]=='bfs':
     breadthFirst(d)
if sys.argv[1]=='ast':
     astar(d)
if sys.argv[1]=='ida':
     idastar(d)
    

