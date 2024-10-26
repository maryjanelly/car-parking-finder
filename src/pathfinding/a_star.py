import heapq

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
    
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def find_path(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_pos = (current[0] + dx, current[1] + dy)
                if 0 <= next_pos[0] < self.rows and 0 <= next_pos[1] < self.cols and self.grid[next_pos[0]][next_pos[1]] == 0:
                    new_cost = cost_so_far[current] + 1
                    if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                        cost_so_far[next_pos] = new_cost
                        priority = new_cost + self.heuristic(goal, next_pos)
                        heapq.heappush(open_list, (priority, next_pos))
                        came_from[next_pos] = current

        path = []
        while goal:
            path.append(goal)
            goal = came_from.get(goal)
        path.reverse()
        return path if path[0] == start else None
