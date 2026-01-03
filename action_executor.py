from nav2_simple_commander.robot_navigator import BasicNavigator

class ActionExecutor:
    def __init__(self, node):
        self.node = node
        self.navigator = BasicNavigator()

    def execute(self, action):
        if action["type"] == "NAVIGATE":
            self.navigator.goToPose(
                self.build_pose(action["target"])
            )

        elif action["type"] == "WAIT":
            self.node.get_logger().info("Waiting...")

        elif action["type"] == "STOP":
            self.navigator.cancelTask()

    def build_pose(self, target):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = target[0]
        pose.pose.position.y = target[1]
        pose.pose.orientation.w = 1.0
        return pose
