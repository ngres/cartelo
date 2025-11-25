import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperTeleoperation(Node):
    """
    A simple node to control a robotic gripper using joystick inputs.
    """

    def __init__(self):
        super().__init__("gripper_teleoperation")

        self.gripper_action_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.last_button_cmds = None

        self.joy_topic = self.declare_parameter("joy_topic", "joy").value
        if not self.joy_topic:
            self.get_logger().error("joy_topic is not set")
            sys.exit(1)

        self.sub = self.create_subscription(
            Joy, self.joy_topic, self._joystick_event_callback, 1
        )
        
    def _joystick_event_callback(self, msg):
        if len(msg.buttons) == 0:
            return
        
        if self.last_button_cmds is None:
            self.last_button_cmds = msg.buttons
            return
        
        if msg.buttons[0] == 1 and self.last_button_cmds[0] == 0:
            try:
                self.get_logger().info("Open gripper")
                self._send_gripper_position(-1.0)
            except Exception as e:
                self.get_logger().error(f"Failed to open gripper: {e}")
        elif msg.buttons[1] == 1 and self.last_button_cmds[1] == 0:
            try:
                self.get_logger().info("Close gripper")
                self._send_gripper_position(1.0)
            except Exception as e:
                self.get_logger().error(f"Failed to close gripper: {e}")
        
        self.last_button_cmds = msg.buttons
                
    def _send_gripper_position(self, position: float, max_effort: float = 5.0):
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        return self.gripper_action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    node = GripperTeleoperation()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(1)