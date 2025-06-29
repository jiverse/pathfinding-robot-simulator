# Grid Setup
grid = [
    [0, 0, 0, 0, 1],
    [1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [1, 0, 0, 1, 0]
]

start = (0, 0)
goal = (4, 4)

ROWS, COLS = len(grid), len(grid[0])

# Path Reconstruction Function
def is_valid(x, y):
    return 0 <= x < ROWS and 0 <= y < COLS and grid[x][y] == 0

def reconstruct_path(came_from, end):
    path = []
    while end in came_from:
        path.append(end)
        end = came_from[end]
    path.append(start)
    return path[::-1]

# Implemented dfs and bfs
from collections import deque

def bfs():
    queue = deque([start])
    visited = set([start])
    came_from = {}

    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return reconstruct_path(came_from, goal)
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny) and (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny))
                came_from[(nx, ny)] = (x, y)
    return []
def dfs():
    stack = [start]
    visited = set()
    came_from = {}

    while stack:
        x, y = stack.pop()
        if (x, y) == goal:
            return reconstruct_path(came_from, goal)
        if (x, y) in visited:
            continue
        visited.add((x, y))
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny) and (nx, ny) not in visited:
                stack.append((nx, ny))
                came_from[(nx, ny)] = (x, y)
    return []

# Implemented dijkstra
import heapq
def dijkstra():
    heap = [(0, start)]
    dist = {start: 0}
    came_from = {}

    while heap:
        cost, (x, y) = heapq.heappop(heap)
        if (x, y) == goal:
            return reconstruct_path(came_from, goal)
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny):
                new_cost = cost + 1
                if (nx, ny) not in dist or new_cost < dist[(nx, ny)]:
                    dist[(nx, ny)] = new_cost
                    heapq.heappush(heap, (new_cost, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)
    return []

# Implemented A*
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan Distance

def a_star():
    heap = [(heuristic(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}

    while heap:
        f, cost, (x, y) = heapq.heappop(heap)
        if (x, y) == goal:
            return reconstruct_path(came_from, goal)
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if is_valid(nx, ny):
                new_g = cost + 1
                if (nx, ny) not in g_score or new_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_g
                    f_score = new_g + heuristic((nx, ny), goal)
                    heapq.heappush(heap, (f_score, new_g, (nx, ny)))
                    came_from[(nx, ny)] = (x, y)
    return []

# Visualized the Path with Animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def animate_path(path):
    mat = np.array(grid, dtype=float)
    fig, ax = plt.subplots()
    img = ax.imshow(mat, cmap='gray_r')

    def update(i):
        if i < len(path):
            x, y = path[i]
            mat[x][y] = 0.5
            img.set_data(mat)
        return [img]

    ani = animation.FuncAnimation(fig, update, frames=len(path), interval=300, repeat=False)
    plt.title("Pathfinding Animation")
    plt.show()

# Combined Everything & Compare
import time

algos = {
    "BFS": bfs,
    "DFS": dfs,
    "Dijkstra": dijkstra,
    "A*": a_star
}

for name, algo in algos.items():
    t0 = time.time()
    path = algo()
    t1 = time.time()
    print(f"{name}: Path Length = {len(path)}, Time = {t1 - t0:.5f} sec")
    animate_path(path)
