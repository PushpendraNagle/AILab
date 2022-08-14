from collections import deque
import numpy as np
import sys

sys.setrecursionlimit(10**3)
print(sys.getrecursionlimit())

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
    for i in range(3):
        for j in range(3):
            if grid[i][j] == 0:
                return i, j

def bfs(grid):
    dis = dict()
    q = deque()
    # Adding intital grid to the queue
    q.append(grid)
    dis[grid] = 0
    while q:
        curr_grid = q.popleft()
        curr_dis = dis[curr_grid]
        x, y = find_blank_position(curr_grid)
        for dx, dy in dir:
            next_x, next_y = x + dx, y + dy
            if next_x>=0 and next_x<3 and next_y>=0 and next_y<3:
                grid = [list(item) for item in curr_grid]
                # Moving blank space to next_x, next_y position
                grid[x][y], grid[next_x][next_y] = grid[next_x][next_y], grid[x][y]
                grid = tuple(tuple(i) for i in grid)
                if grid not in dis:
                    dis[grid] = curr_dis + 1
                    if grid == target_grid:
                        return dis[grid]
                    q.append(grid)
    return -1

def dfs(curr_grid, vis=set()):
    if curr_grid == target_grid:
        return 0
    vis.add(curr_grid)
    x, y = find_blank_position(curr_grid)
    for dx, dy in dir:
        next_x, next_y = x + dx, y + dy
        if next_x>=0 and next_x<3 and next_y>=0 and next_y<3:
            grid = [list(item) for item in curr_grid]
            # Moving blank space to next_x, next_y position
            grid[x][y], grid[next_x][next_y] = grid[next_x][next_y], grid[x][y]
            grid = tuple(tuple(i) for i in grid)
            if grid not in vis:
                steps = dfs(grid, vis)
                if steps != -1:
                    return steps + 1
    return -1

def main():
    # Genrate random input
    grid = generate_random_grid()
    print('Randomly generated grid:')
    print(grid)
    
    bfs_steps = bfs(grid)
    if bfs_steps == -1:
        print('Solution not possible.')
    else:
        print('Number of steps required to reach the solution using BFS:', bfs_steps)

    dfs_steps = dfs(grid)
    if dfs_steps == -1:
        print('Solution not possible.')
    else:
        print('Number of steps required to reach the solution using DFS:', dfs_steps)

if __name__=='__main__':
    main()
