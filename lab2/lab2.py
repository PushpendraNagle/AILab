# -*- coding: utf-8 -*-
"""lab2

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1i6cfJ_90mzBSqUbNaxW5Es3gsuUVgyCM
"""

from collections import deque
import numpy as np
import resource, sys
import time
from queue import PriorityQueue

# Increasing resource limit to avoid exceeding recursion limit
resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
sys.setrecursionlimit(10**6)

# 0 will represent blank space in the grid
target_grid = ((1, 2, 3),
               (4, 5, 6),
               (7, 8, 0))

# Storing directions to move the blank space in a tuple
dir = ((1, 0), (-1, 0), (0, 1), (0, -1))

def generate_random_grid():
    # Generating a random permutation from digits 0 to n*m-1
    perm = np.random.permutation(9)
    grid = ((perm[0], perm[1], perm[2]),
            (perm[3], perm[4], perm[5]),
            (perm[6], perm[7], perm[8]))
    return grid

def find_blank_position(grid):
    # Function to find row and column number of blank cell in the grid
    for i in range(0,3):
        for j in range(0,3):
            if grid[i][j] == 0:
                return i, j

def heuristic(grid,type):
  if type==1:
    return 0
  elif type == 2:
    count=0
    for i in range(0,3):
      for j in range(0,3):
        if grid[i][j]!=i*3+j+1:
          count = count +1
          # print(i," ",j)
    return count
  elif type == 3:
    dist=0;
    for i in range(0,3):
      for j in range(0,3):
        if grid[i][j]==0:
          continue
        orx=(int)((grid[i][j]-1)/3)
        ory=(grid[i][j]-1)%3
        d=(abs(i-orx)+abs(j-ory))
        dist+=d
        # print(d)
    return dist
  elif type==4:
    dist=0;
    for i in range(0,3):
      for j in range(0,3):
        if grid[i][j]==0:
          continue
        orx=(int)((grid[i][j]-1)/3)
        ory=(grid[i][j]-1)%3
        # print(orx," ",ory)
        d=np.sqrt((i-orx)*(i-orx)+(j-ory)*(j-ory))
        dist+=d
        # print(d)
    return dist

def a_star(grid,type):
  print('Heuristic ',type)
  start_time = time.time()
  dist = dict()
  parent = dict()
  open_list  = PriorityQueue()
  # insert into queue
  open_list.put([heuristic(grid,type), grid])
  closed_list = set()
  dist[grid] = 0
  parent[grid] = grid
  while open_list.qsize() > 0:
    prior, curr_grid = open_list.get()
    print(curr_grid)
    if curr_grid == target_grid:
      print("Yayy! Target grid reached!")
      end_time = time.time()
      print('Start State:')
      print(grid)
      print('Goal state')
      
      print(target_grid)
      print('Total number of states to optimal path = ',dist[target_grid]+1)
      print('No of states explored = ', len(closed_list))
      
      
      g = target_grid
      path = []
      while g!=grid:
        path.insert(0,g)
        g = parent[g]
      path.insert(0,g)
      for x in path:
        print(x)
        if x==target_grid:
          continue
        print('                |')
        print('                |')
        print('                V')
      print('Optimal path cost = ', dist[target_grid])
      print('Time taken for execution = ', end_time-start_time)       
      return
    bx, by = find_blank_position(curr_grid)
    for dx, dy in dir:
      next_x, next_y = bx + dx, by + dy
      if next_x>=0 and next_x<3 and next_y>=0 and next_y<3:
        # tuples are immutable in python therefore we need to 
        # store grid in list swap the blank space with adjacent
        # cell and then store it back in tuple.
        # Tuples are used to store grid state because these are
        # hashable and can be stored in python set or dictionary
        grid_t = [list(item) for item in curr_grid]
        # Moving blank space to next_x, next_y position
        grid_t[bx][by], grid_t[next_x][next_y] = grid_t[next_x][next_y], grid_t[bx][by]
        grid_t = tuple(tuple(i) for i in grid_t)
       

        if grid_t not in closed_list and (grid_t not in dist or dist[grid_t]>dist[curr_grid]+1):
          dist[grid_t] = dist[curr_grid]+1
          parent[grid_t] = curr_grid
          open_list.put([heuristic(grid_t,type)+dist[grid_t],grid_t])
        
        closed_list.add(curr_grid)
          
    
  print('Oops! Failed to reach the target grid!')
  end_time = time.time()
  print('Start State:')
  print(grid)
  print('Goal state')
  print(target_grid)
  print('No of states explored = ', len(closed_list))
  print('Time taken for execution = ', end_time-start_time)
  return -1

def main():
    # Generate random input
    grid = generate_random_grid()
    
    print('Randomly generated grid:')
    print(grid)

    # grid = ((1,2,3),(4,5,6),(0,7,8))

    a_star(grid,4)
    print("\n\n")
    a_star(grid,3)
    print("\n\n")
    a_star(grid,2)
    print("\n\n")  
    a_star(grid,1)
    
if __name__=='__main__':
    main()
