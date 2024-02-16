from geometry_msgs.msg import TransformStamped, Quaternion

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from px4_msgs.msg import VehicleOdometry

import numpy as np


def quaternion_multiply(q1, q2):
    q_w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q_x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q_y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q_z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]

    q = np.array([q_w, q_x, q_y, q_z])
    q = q/np.linalg.norm(q)

    return q

def quaternion_inverse(q):
    q = np.array([q[0], -q[1], -q[2], -q[3]])
    q = q/np.linalg.norm(q)

    return q

class PX4Broadcaster(Node):
    def __init__(self):
        super().__init__('px4_broadcaster')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Set tf_broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster2 = TransformBroadcaster(self)

        # Set subscriber
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.broadcast_callback,
            qos_profile)
        self.vehicle_odometry_sub

        self.timer = self.create_timer(0.02, self.cmdloop_callback)
        
    def broadcast_callback(self, msg):
        # odom to base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # ns = msg.timestamp * 1000
        # t.header.stamp = Time(nanoseconds=ns).to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        # Change NED to ENU
        t.transform.translation.x = float(msg.position[1])
        t.transform.translation.y = float(msg.position[0])
        t.transform.translation.z = 0.

        q_NEDtoENU = np.array([0, -np.sqrt(2)/2, -np.sqrt(2)/2, 0.])
        q_FRDtoFLU = np.array([0, 1, 0, 0])

        q = quaternion_multiply(quaternion_multiply(quaternion_inverse(q_NEDtoENU), msg.q), q_FRDtoFLU)

        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])

        self.tf_broadcaster.sendTransform(t)

        # base_footprint to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # t.header.stamp = Time(nanoseconds=ns).to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'

        # Change NED to ENU
        t.transform.translation.x = 0.
        t.transform.translation.y = 0.
        t.transform.translation.z = -float(msg.position[2])

        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

    def cmdloop_callback(self):
        # Set instead of tf_static_broadcaster(map to odom)
        #TODO: change to launch file
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.tf_broadcaster2.sendTransform(t)

def main():
    rclpy.init()

    node = PX4Broadcaster()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()