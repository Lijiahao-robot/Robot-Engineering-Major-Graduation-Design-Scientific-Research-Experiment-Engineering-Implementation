import heapq
import time

# A* path planning
def astar(start, goal, grid):
    rows, cols = len(grid), len(grid[0])
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g = {start: 0}

    def h(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nxt = (current[0] + dx, current[1] + dy)
            if 0 <= nxt[0] < rows and 0 <= nxt[1] < cols and grid[nxt[0]][nxt[1]] == 0:
                new_cost = g[current] + 1
                if nxt not in g or new_cost < g[nxt]:
                    g[nxt] = new_cost
                    f = new_cost + h(nxt)
                    heapq.heappush(open_list, (f, nxt))
                    came_from[nxt] = current

    return None

# Real-time obstacle detection (pseudo)
def detect_obstacle():
    # 这里可接入超声波、激光雷达等传感器
    return False

# Robot main loop
grid = [
    [0,0,0,0],
    [0,1,1,0],
    [0,0,0,0]
]

path = astar((0,0), (2,3), grid)
print("Planned path:", path)

for step in path:
    if detect_obstacle():
        print("Obstacle detected! Replanning...")
        path = astar(step, (2,3), grid)
    print("Moving to:", step)
    time.sleep(0.5)
