class StateManager:
    def __init__(self, node):
        self.node = node
        self.pose = None
        self.objects = []

        node.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def get_context(self):
        return {
            "pose": self.pose,
            "objects": self.objects,
            "task": "go to target and wait"
        }
