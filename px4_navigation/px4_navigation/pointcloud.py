import open3d
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from px4_msgs.msg import VehicleOdometry

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

def pointcloud2_to_open3d(msg):
    """
    Header header
    uint32 height
    uint32 weight
    PointField[] fields
    bool is_bigendian
    uint32 point_step
    uint32 row_step
    uint8[] data
    bool is_dense
    """

    field_names = [field.name for field in msg.fields]
    cloud_data = list(point_cloud2.read_points(msg, skip_nans=True, field_names=field_names))

    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None
    
    xyz = [(x,y,z) for x,y,z,rgb in cloud_data]

    open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    return open3d_cloud



class DepthPointsSubscriber(Node):
    def __init__(self):
        super().__init__('depth_points_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.depth_points_sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.depth_points_callback,
            10)
        self.depth_points_sub

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile)
        self.vehicle_odometry_sub

        self.points_publisher = self.create_publisher(PointCloud2, "/depth_points", 10)

        self.height = 0.0
        self.max_depth = 65.535 #TODO: Set to ZED2i
        self.margin = 0.5
        self.frame_id = 'base_link'

        self.R = np.array([[0, 0, 1, 0],
                           [-1, 0, 0, 0],
                           [0, -1, 0, 0],
                           [0, 0, 0, 1]])

    def depth_points_callback(self, msg):
        pcd = pointcloud2_to_open3d(msg)
        points = np.asarray(pcd.points)
        mask = np.logical_and(points[:,1] < (self.height - self.margin), points[:,2] < self.max_depth)
        pcd.points = open3d.utility.Vector3dVector(points[mask])
        pcd = pcd.transform(self.R)

        pc2_cloud = self.open3d_to_pointcloud2(pcd, msg)
        self.points_publisher.publish(pc2_cloud)

        # open3d.io.write_point_cloud('points.pcd', pcd)

    def odom_callback(self, msg):
        self.height = -msg.position[2]

    def open3d_to_pointcloud2(self, pcd, msg):
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frame_id

        fields = FIELDS_XYZ
        points = np.asarray(pcd.points)

        return point_cloud2.create_cloud(header, fields, points)

def main():
    rclpy.init()

    node = DepthPointsSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()