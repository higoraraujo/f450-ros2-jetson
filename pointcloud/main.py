import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile
import message_filters

class PointCloudFusionNode(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion_node')
        
        qos_profile = QoSProfile(depth=10)
        
        # Subscribing to the point cloud, IMU, and GPS topics using message_filters for ROS2
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', qos_profile)
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data', qos_profile)
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, '/gps/fix', qos_profile)

        # Synchronize the topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub, self.gps_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, pointcloud, imu, gps):
        # Here you can process the synchronized data
        self.get_logger().info('Synchronized point cloud, imu, and gps data')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
