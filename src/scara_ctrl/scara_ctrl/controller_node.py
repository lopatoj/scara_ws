import rclpy
import numpy as np
from .kinematics import Kinematics

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.dt: float = self.declare_parameter("dt", 0.01).value

        self.declare_parameter("k_e", 100.0)
        self.declare_parameter("k_d", 35.0)
        self.declare_parameter("k_i", 15.0)

        self.target_pos = np.zeros(shape=(3,))
        self.curr_pos = np.zeros(shape=(3,))
        self.prev_error = np.zeros(shape=(3,))
        self.ierror = np.zeros(shape=(3,))

        self.joint_states = JointState()
        self.ctrl = Float64MultiArray()
        self.kinematics = Kinematics()

        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10
        )
        self.pos_sub = self.create_subscription(
            Vector3, "/pos_cmd", self._pos_callback, 10
        )
        self.ctrl_pub = self.create_publisher(
            Float64MultiArray, "/ctrl_cmd", 10
        )
        self.create_timer(self.dt, self._step_callback)

    def _step_callback(self):
        if len(self.joint_states.name) == 0:
            return

        self.ctrl.data = self.get_ctrl()
        self.ctrl_pub.publish(self.ctrl)

    def _pos_callback(self, msg: Vector3):
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def _joint_states_callback(self, msg: JointState):
        self.joint_states = msg

    def get_ctrl(self):
        k_e = self.get_parameter("k_e").value
        k_i = self.get_parameter("k_i").value
        k_d = self.get_parameter("k_d").value

        joint_pos = np.array(self.joint_states.position)
        # joint_vel = np.array(self.joint_states.velocity)
        self.curr_pos = self.kinematics.forward(joint_pos)

        error = self.target_pos - self.curr_pos
        self.ierror = self.ierror + (error * self.dt)
        derror = (error - self.prev_error) / self.dt
        self.prev_error = error

        pid = k_e * error + k_i * self.ierror + k_d * derror

        J = self.kinematics.jacobian(joint_pos)

        return (J.T @ (pid)).tolist()


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
