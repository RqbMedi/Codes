import sys
from random import randint
from BaseAI import BaseAI
from Grid import Grid
import numpy as np
import math

class PlayerAI(BaseAI):
        def getMove(self, grid):
                newgrid = []
                for i in range(4):
      ##Create a copy of the grid
                     newgrid.extend(grid.map[i])
      ##Collect the available children and their respective moves
                [child,move] = getAvailableChildren(newgrid)  
      ##Initialize the path to -inf and the dir to 0
                p = -np.inf
                dir = 0
      ##Loop necessary for finding the best choice
                for i in range(len(child)):
                        max_v = -np.inf
                        maxdepth = 3
      ##The necessary move to get the child grid from the parent grid          
                        c = child[i]
                        m = move[i]
      ##Minimax Algorithm wiht alpha-beta pruning
                        max_v = MINIMAXALPHABETA(c, maxdepth,-np.inf,np.inf, True)
                        if max_v > p:
                            dir = m
                            p = max_v
                return dir

##################################################################################################
#######################Minimaxab####################################################################
##################################################################################################
def MINIMAXALPHABETA(grid, maxdepth, alpha, beta, maximize):
 ##If we are in 0 then apply heuristic1 to the grid
    if maxdepth == 0:
        return heuristic1(grid)
 ##If we are not able to move then apply heuristic1 to the grid
    if not canMove(grid):
        return heuristic1(grid)
 ##If True
    if maximize:
        v = -np.inf
        [chil, mov] = getAvailableChildren(grid)
        for child in chil:
            v = max(v,MINIMAXALPHABETA(child,maxdepth-1,alpha,beta,False))
            if v >= beta:
                return v
            alpha = max(alpha,v)
        return v
 ##If False
    else:
        cells = [i for i, x in enumerate(grid) if x == 0]
        chil = []
     ##All the different choices which the opponent aims to minimize
        for c in cells:
            newnewgrid = list(grid)
            newnewgrid[c]=2
            chil.append(newnewgrid)
            newnewgrid = list(grid)
            newnewgrid[c]=4
            chil.append(newnewgrid)
        v = np.inf
        for child in chil:
            v = min(v,MINIMAXALPHABETA(child,maxdepth-1,alpha,beta,True))
            if v <= alpha:
                return v
            beta = min(beta,v)
        return v

###################################################################################
################################Heuristic##########################################
###################################################################################       
def heuristic1(grid):
   cell = [i for i, x in enumerate(grid) if x == 0]
   maxTile= max(grid)
   if maxTile == 1024:
       value = 1000*maxTile
   else:
       value=1000*len(cell)
   return(value)    

##################################################################################################
#######################Other Functions####################################################################
##################################################################################################
def getAvailableChildren(grid):
    ##gets all children and the moving directions
    allmoves = [0,1,2,3]
    children = []
    moving = []
    for m in allmoves:
        gridcopy = list(grid)
        moved = move(gridcopy, m)
        ##move method returns True if moved and makes the change to gridcopy itself
        if moved == True:
            children.append(gridcopy)
            moving.append(m)
    return [children,moving]
def merge(cells):
    ##merges the cells and sends back in order to be inserted
    if len(cells) <= 1:
        return cells
    i = 0
    while i < len(cells)-1:
        if cells[i] == cells[i+1]:
            cells[i] *= 2
            del cells[i+1]
        i += 1
def move(grid, direction):
    ##if there is a move it is changed and return is True
    moved = False
    if direction == 0:
        ##UP
        for i in range(4):
            cells = []
            ##cells has all elements for a column from top to bottom
            for j in range(i,i+13,4):
                cell = grid[j]
                if cell != 0:
                    cells.append(cell)
            merge(cells)
            for j in range(i,i+13,4):
                value = cells.pop(0) if cells else 0
                if grid[j] != value:
                    moved = True
                grid[j] = value
        return moved
    elif direction == 1:
        ##DOWN
        for i in range(4):
            cells = []
            ##cells has all elements of column from bottom to top
            for j in range(i+12,i-1,-4):
                cell = grid[j]
                if cell != 0:
                    cells.append(cell)
            merge(cells)
            for j in range(i+12,i-1,-4):
                value = cells.pop(0) if cells else 0
                if grid[j] != value:
                    moved = True
                grid[j] = value
        return moved
    elif direction == 2:
        ##LEFT
        for i in [0,4,8,12]:
            cells = []
            ##cells has all elements of a row from left to right
            for j in range(i,i+4):
                cell = grid[j]
                if cell != 0:
                    cells.append(cell)
            merge(cells)
            for j in range(i,i+4):
                value = cells.pop(0) if cells else 0
                if grid[j] != value:
                    moved = True
                grid[j] = value
        return moved
    elif direction == 3:
        ##RIGHT
        for i in [3,7,11,15]:
            cells = []
            ##cells has all elements of a row from right to left
            for j in range(i,i-4,-1):
                cell = grid[j]
                if cell != 0:
                    cells.append(cell)
            merge(cells)
            for j in range(i,i-4,-1):
                value = cells.pop(0) if cells else 0
                if grid[j] != value:
                    moved = True
                grid[j] = value
        return moved
def canMove(grid):
    if 0 in grid:
        ##if there is an empty space in the grid
        return True
    for i in range(16):
        if (i+1)%4!=0:
            ##for all elements except the last column
            ##if any element has same right element
            if grid[i]==grid[i+1]:
                return True
        if i<12:
            ##for all except last row elements
            ##if any element has same below element
            if grid[i]==grid[i+4]:
                return True
    return False
