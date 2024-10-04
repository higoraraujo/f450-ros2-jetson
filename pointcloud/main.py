import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix
from message_filters import ApproximateTimeSynchronizer, Subscriber

class PointCloudFusionNode(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion_node')
        
        # Subscribing to the point cloud, IMU, and GPS topics
        self.pc_sub = Subscriber(self, PointCloud2, '/zed/zed_node/point_cloud/cloud_registered')
        self.imu_sub = Subscriber(self, Imu, '/imu/data')
        self.gps_sub = Subscriber(self, NavSatFix, '/gps/fix')

        # Synchronize the topics
        self.ts = ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub, self.gps_sub], queue_size=10, slop=0.1)
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
