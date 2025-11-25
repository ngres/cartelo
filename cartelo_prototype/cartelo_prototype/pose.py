import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformStamped
import numpy as np
from geometry_msgs.msg import Quaternion, Transform
import sys
import tf_transformations

class PoseTeleoperation(Node):
    """
    Allows teleoperation of a robot's end-effector pose using a 3D tracker.

    Pressing joystick button 7 will set set the reference frame of the tracker to the current position of the end-effector.
    Keeping button 7 pressed and moving the tracker will move the end-effector relative to this position.
    """

    def __init__(self):
        super().__init__("pose_teleoperation")

        self.frame_transform = None

        self.virtual_world_frame_id = self.declare_parameter("virtual_world_frame_id", "teleop_world").get_parameter_value().string_value
        self.controller_frame_id = self.declare_parameter("controller_frame_id", "LHR-FF29DD46").get_parameter_value().string_value
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_link").get_parameter_value().string_value
        self.end_effector_frame_id = self.declare_parameter("end_effector_frame_id", "gripper_link").get_parameter_value().string_value

        self.last_button_cmds = None

        self.transform_delta: Transform | None = None

        self.tracker_tf_buffer = tf2_ros.Buffer()
        self.tracker_tf_listener = tf2_ros.TransformListener(self.tracker_tf_buffer, self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.joy_topic = self.declare_parameter("joy_topic", "joy").get_parameter_value().string_value

        self.frame_transform_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Joy, self.joy_topic, self.joystick_event_callback, 1
        )

        self.end_effector_pose_topic = self.declare_parameter("target_pose_topic", "/target_end_effector_pose").get_parameter_value().string_value

        self.pub = self.create_publisher(PoseStamped, self.end_effector_pose_topic, 3)

        period = 1.0 / self.declare_parameter("publishing_rate", 100).get_parameter_value().integer_value
        self.pose_timer = self.create_timer(period, self.publish_target_pose)

        self.frame_timer = self.create_timer(0.1, self.broadcast_frame_transform)
        
    def joystick_event_callback(self, msg):
        if len(msg.buttons) == 0:
            return
        
        if self.last_button_cmds is None:
            self.last_button_cmds = msg.buttons
            return
        
        # log all buttons that have been pressed
        for i, button in enumerate(msg.buttons):
            if button == 1 and self.last_button_cmds[i] == 0:
                self.get_logger().info(f"Button {i} pressed")

        if msg.buttons[6] == 1 and self.last_button_cmds[6] == 0:
            try:
                self.get_logger().info("Calibrate frame")
                self.calibrate_frame()
            except Exception as e:
                self.get_logger().error(f"Failed to calibrate frame: {e}")

        if msg.buttons[7] == 0 and self.last_button_cmds[7] == 1:
            try:
                self.get_logger().info("Stop teleoperation")
                self.stop_teleoperation()
            except Exception as e:
                self.get_logger().error(f"Failed to stop teleoperation: {e}")
            
        elif msg.buttons[7] == 1 and self.last_button_cmds[7] == 0:
            try:
                self.get_logger().info("Start teleoperation")
                self.start_teleoperation()
            except Exception as e:
                self.get_logger().error(f"Failed to start teleoperation: {e}")
        
        self.last_button_cmds = msg.buttons

    def calibrate_frame(self):
        self.frame_transform = self.get_frame_transform()
        if self.frame_transform is None:
            self.get_logger().error("Could not get frame transform")
            return
        
        self.broadcast_frame_transform()

    def start_teleoperation(self):
        time = rclpy.time.Time()
        b_T_e, b_T_c = None, None
        try:
            b_T_e = self.tf_buffer.lookup_transform(
                
                self.base_frame_id,
                self.end_effector_frame_id,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame_id} to {self.end_effector_frame_id}: {ex}')
            return
        try:
            b_T_c = self.tf_buffer.lookup_transform(
                
                self.base_frame_id,
                self.controller_frame_id,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame_id} to {self.controller_frame_id}: {ex}')
            return
        
        if b_T_e is None or b_T_c is None:
            return
        
        self.transform_delta = Transform()
        self.transform_delta.translation.x = b_T_c.transform.translation.x - b_T_e.transform.translation.x
        self.transform_delta.translation.y = b_T_c.transform.translation.y - b_T_e.transform.translation.y
        self.transform_delta.translation.z = b_T_c.transform.translation.z - b_T_e.transform.translation.z
        c_quat = self._quaternion_to_array(b_T_c.transform.rotation)
        e_quat = self._quaternion_to_array(b_T_e.transform.rotation)
        delta_quat = tf_transformations.quaternion_multiply(tf_transformations.quaternion_conjugate(e_quat), c_quat)
        self.transform_delta.rotation.x = delta_quat[0]
        self.transform_delta.rotation.y = delta_quat[1]
        self.transform_delta.rotation.z = delta_quat[2]
        self.transform_delta.rotation.w = delta_quat[3]

    def stop_teleoperation(self):
        self.transform_delta = None

    
    def get_frame_transform(self) -> TransformStamped | None:
        """
        Get a transform between the virtual world frame and the robot base frame.
        """

        v_T_c = None

        try:
            v_T_c = self.tf_buffer.lookup_transform(
                self.virtual_world_frame_id,
                self.controller_frame_id,
                rclpy.time.Time())
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.virtual_world_frame_id} to {self.controller_frame_id}: {ex}')
            return None

        if v_T_c is None:
            return None
        
        v_T_b = TransformStamped()
        v_T_b.header.stamp = v_T_c.header.stamp
        v_T_b.header.frame_id = v_T_c.header.frame_id
        v_T_b.child_frame_id = self.base_frame_id

        v_T_b_mat = self._transform_to_matrix(v_T_c.transform)
        # [TODO] always ensure that the z-axis of the base frame points upwards
        conversion_mat = [
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ]

        v_T_b_mat = tf_transformations.concatenate_matrices(v_T_b_mat, conversion_mat)
        v_T_b.transform = self._matrix_to_transform(v_T_b_mat)

        return v_T_b

    def _transform_to_matrix(self, t: Transform) -> np.ndarray:
        return tf_transformations.concatenate_matrices(
            tf_transformations.translation_matrix([t.translation.x, t.translation.y, t.translation.z]),
            tf_transformations.quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        )
        
    def _matrix_to_transform(self, mat: np.ndarray) -> Transform:
        t = Transform()
        trans = tf_transformations.translation_from_matrix(mat)
        quat = tf_transformations.quaternion_from_matrix(mat)
        t.translation.x, t.translation.y, t.translation.z = trans
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = quat
        return t

    def _quaternion_to_array(self, q: Quaternion) -> np.ndarray:
        return np.array([q.x, q.y, q.z, q.w])
    
    def broadcast_frame_transform(self):
        if self.frame_transform is None:
            return None

        self.frame_transform.header.stamp = self.get_clock().now().to_msg()
        
        self.frame_transform_broadcaster.sendTransform(self.frame_transform)

    def publish_target_pose(self):
        if self.frame_transform is None or self.transform_delta is None:
            return None
        
        b_T_c = None

        try:
            b_T_c = self.tf_buffer.lookup_transform(
                self.base_frame_id,
                self.controller_frame_id,
                rclpy.time.Time())
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame_id} to {self.controller_frame_id}: {ex}')
            return None
        
        if b_T_c is None:
            return None
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.pose.position.x = b_T_c.transform.translation.x - self.transform_delta.translation.x
        msg.pose.position.y = b_T_c.transform.translation.y - self.transform_delta.translation.y
        msg.pose.position.z = b_T_c.transform.translation.z - self.transform_delta.translation.z
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = tf_transformations.quaternion_multiply(self._quaternion_to_array(b_T_c.transform.rotation), tf_transformations.quaternion_conjugate(self._quaternion_to_array(self.transform_delta.rotation)))
        
        try:
            self.pub.publish(msg)
        except Exception:
            # Swallow 'publish() to closed topic' error.
            # This rarely happens on killing this node.
            pass
        


def main(args=None):
    rclpy.init(args=args)

    node = PoseTeleoperation()
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