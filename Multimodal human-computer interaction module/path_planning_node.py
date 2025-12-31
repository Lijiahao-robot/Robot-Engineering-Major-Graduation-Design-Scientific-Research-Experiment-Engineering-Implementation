#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class AStarPlanner:
    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.resolution = resolution
        self.origin = origin
        self.height, self.width = grid.shape
        self.movements = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    def grid2world(self, x, y):
        wx = x * self.resolution + self.origin[0]
        wy = y * self.resolution + self.origin[1]
        return wx, wy

    def world2grid(self, wx, wy):
        x = int((wx - self.origin[0]) / self.resolution)
        y = int((wy - self.origin[1]) / self.resolution)
        return x, y

    def plan(self, start_world, goal_world):
        start = self.world2grid(*start_world)
        goal = self.world2grid(*goal_world)
        if self.grid[start[0], start[1]] > 50 or self.grid[goal[0], goal[1]] > 50:
            return None
        # A* 核心逻辑
        open_set = {start: 0 + self.heuristic(start, goal)}
        came_from = {}
        g_score = {start: 0}

        while open_set:
            current = min(open_set, key=open_set.get)
            if current == goal:
                return self.reconstruct_path(came_from, current)
            del open_set[current]
            for dx, dy in self.movements:
                neighbor = (current[0]+dx, current[1]+dy)
                if 0<=neighbor[0]<self.height and 0<=neighbor[1]<self.width:
                    if self.grid[neighbor[0], neighbor[1]] > 50:
                        continue
                    tentative_g = g_score[current] + np.hypot(dx, dy)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        open_set[neighbor] = tentative_g + self.heuristic(neighbor, goal)
        return None

    def heuristic(self, a, b):
        return np.hypot(a[0]-b[0], a[1]-b[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return [(self.grid2world(x, y)) for x, y in reversed(path)]

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning_node")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.start_world = (0.0, 0.0)
        self.goal_world = (5.0, 5.0)  # 可通过参数动态设置
        self.get_logger().info("A* 路径规划节点已启动")

    def map_callback(self, msg):
        grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        planner = AStarPlanner(grid, msg.info.resolution, (msg.info.origin.position.x, msg.info.origin.position.y))
        path_points = planner.plan(self.start_world, self.goal_world)
        if path_points is not None:
            path_msg = Path()
            path_msg.header = msg.header
            for x, y in path_points:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"规划路径完成，共 {len(path_points)} 个点")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
