import rclpy
from rclpy.node import Node
from vla_decision.state_manager import StateManager
from vla_decision.prompt_builder import PromptBuilder
from vla_decision.action_executor import ActionExecutor

class VLADecisionNode(Node):
    def __init__(self):
        super().__init__('vla_decision_node')

        self.state = StateManager(self)
        self.executor = ActionExecutor(self)
        self.prompt_builder = PromptBuilder()

        self.timer = self.create_timer(1.0, self.decision_loop)  # 1 Hz

    def decision_loop(self):
        context = self.state.get_context()
        prompt = self.prompt_builder.build(context)

        action = self.call_vla_model(prompt)

        if action["confidence"] < 0.6:
            action = {"type": "WAIT", "duration": 1.0}

        self.executor.execute(action)

    def call_vla_model(self, prompt):
        """
        这里可以替换为：
        - RoboVLM
        - VLA Action Token Policy
        - LLM API
        """
        return {
            "type": "NAVIGATE",
            "target": [2.0, 1.5],
            "confidence": 0.9
        }


def main():
    rclpy.init()
    node = VLADecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()
