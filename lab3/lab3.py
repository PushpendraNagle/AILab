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

# fn to calculate heuristics
def heuristic_value(grid,type):
  if type==1:                           
    count=0                           #count of misplaced tiles from their destined position.
    for i in range(0,3):
      for j in range(0,3):
        if grid[i][j]!=i*3+j+1:
          count = count +1
    return count
  elif type == 2:
    dist=0;                           #sum of Manhattan distance of each tiles from the goal position
    for i in range(0,3):
      for j in range(0,3):
        if grid[i][j]==0:
          continue
        orx=(int)((grid[i][j]-1)/3)
        ory=(grid[i][j]-1)%3
        d=(abs(i-orx)+abs(j-ory))
        dist+=d
    return dist


class HillClimbing:
  def __init__(self):
    self.visited = set()
    self.dist = dict()                   # a dictionary to store the distance of states from start state
    self.parent = dict()                 # a dictionary to store the parent of each state
    self.grid = tuple()

  def print_path(self):
    g = target_grid
    path = []
    while g!=self.grid:
      path.insert(0,g)
      g = self.parent[g]
    path.insert(0,g)
    for x in path:
      print(x)
      if x==target_grid:
        continue
      print('                |')
      print('                |')
      print('                V')

  def success(self):
    print("Solution reached.") 
    end_time = time.time()
    print('Total number of states to optimal path = ',self.dist[target_grid]+1)
    print('No of states explored = ', len(self.visited)) 
    print('Optimal path cost = ', self.dist[target_grid])
    self.print_path()

  def failure(self):
    print("Failed to reach solution.")     
    print('No of states explored = ', len(self.visited))

  def search(self, state, heuristic):
    current = state
    self.dist[current] = 0
    while True:    
      if current == target_grid:
        self.success()
        return
      self.visited.add(current)
      bx, by = find_blank_position(current)
      neighbours = PriorityQueue()
      for dx, dy in dir:                    # expand all the possible next states of the current grid
        next_x, next_y = bx + dx, by + dy
        if next_x>=0 and next_x<3 and next_y>=0 and next_y<3:
          # tuples are immutable in python therefore we need to 
          # store grid in list swap the blank space with adjacent
          # cell and then store it back in tuple.
          # Tuples are used to store grid state because these are
          # hashable and can be stored in python set or dictionary
          grid_t = [list(item) for item in current]
          # Moving blank space to next_x, next_y position
          grid_t[bx][by], grid_t[next_x][next_y] = grid_t[next_x][next_y], grid_t[bx][by]
          grid_t = tuple(tuple(i) for i in grid_t)       
          if grid_t not in self.visited:
            self.dist[grid_t] = self.dist[current]+1
            self.parent[grid_t] = current
            neighbours.put([heuristic_value(grid_t,heuristic)+self.dist[grid_t],grid_t])
      
      if neighbours.qsize() > 0:
        _, best_neighbor = neighbours.get()
        current = best_neighbor
      else:
        self.failure()
        break
    
  def __call__(self, state, heuristic):
    self.grid = state
    self.search(state, heuristic)

def main():
    # Generate random input
    grid = generate_random_grid()
    
    print('Randomly generated grid:')
    print(grid)
    ha1 = HillClimbing()
    print('\n---------------------------------------------------------------------')
    print('Starting Hill Climbing search with heuristic -> h1(n) = number of tiles displaced from their destined position')
    print('Start State:')
    print(grid)
    print('Goal state')    
    print(target_grid)
    solution1 = ha1(grid, heuristic=1)
    ha2 = HillClimbing()
    print('\n---------------------------------------------------------------------')
    print('Starting Hill Climbing search with heuristic -> h2(n) = sum of the Manhattan distance of each tile from the goal position')
    print('Start State:')
    print(grid)
    print('Goal state')    
    print(target_grid)
    solution2 = ha2(grid, heuristic=2)
    
    
if __name__=='__main__':
    main()