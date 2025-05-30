import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import numpy as np

class DataFuse(Node):
    def __init__(self):
        super().__init__('combinedScan')

        self.front_offset_x=0.19
        self.front_offset_y=0.09
        self.back_offset_x=0.17
        self.back_offset_y=0.09


    
        self.tf_broadcast = tf2_ros.TransformBroadcaster(self)

        self.scan1 = None
        self.scan2 = None

        self.create_subscription(LaserScan, 'scan1', self.f, 10)
        self.create_subscription(LaserScan, 'scan2', self.b, 10)

        self.scanCombined = self.create_publisher(LaserScan, 'scan', 10)

        self.timer = self.create_timer(0.005, self.CombinedScan)  

        self.publish_logged = False

    def f(self, msg):
        self.scan1 = msg

    def b(self, msg):
        self.scan2 = msg


    def CombinedScan(self):
        if self.scan1 is None or self.scan2 is None or not self.scan1.angle_increment or not self.scan2.angle_increment:

            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "laser"
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.22
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0

        self.tf_broadcast.sendTransform(tf_msg)



        angle_min = 0.0
        angle_max = 2 * math.pi
        angle_increment = self.scan1.angle_increment
        num = int(math.ceil((angle_max - angle_min) / angle_increment))

        scan_data = LaserScan()
        scan_data.header.frame_id = "laser"
        scan_data.header.stamp = self.get_clock().now().to_msg()
        scan_data.range_min = 0.1
        scan_data.range_max = 12.0
        scan_data.scan_time = 0.01





        
        scan_data.angle_min = angle_min
        scan_data.angle_max = angle_max
        scan_data.angle_increment = angle_increment
        scan_data.ranges = [float(0)] * num

        for i, range_front in enumerate(self.scan1.ranges):
            theta = self.scan1.angle_min + i * self.scan1.angle_increment
            x_sensor = range_front * math.cos(theta)
            y_sensor = range_front * math.sin(theta)

            x_center = -self.front_offset_x + x_sensor
            y_center = -self.front_offset_y + y_sensor

            computed_range = math.hypot(x_center, y_center)
            theta_combined = math.atan2(y_center, x_center)
            if theta_combined < 0:
                theta_combined += 2 * math.pi

            j = int(math.floor(theta_combined / angle_increment))
            if 0 <= j < num and not math.isinf(computed_range):
                scan_data.ranges[j] = max(scan_data.ranges[j], computed_range)

        for i, range_back in enumerate(self.scan2.ranges):
            theta = self.scan2.angle_min + i * self.scan2.angle_increment
            x_sensor = range_back * math.cos(theta)
            y_sensor = range_back * math.sin(theta)

            x_center = self.back_offset_x + x_sensor
            y_center = self.back_offset_y + y_sensor

            computed_range = math.hypot(x_center, y_center)
            theta_combined = math.atan2(y_center, x_center)
            if theta_combined < 0:
                theta_combined += 2 * math.pi

            j = int(math.floor(theta_combined / angle_increment))
            if 0 <= j < num and not math.isinf(computed_range):
                scan_data.ranges[j] = max(scan_data.ranges[j], computed_range)

        self.scanCombined.publish(scan_data)

        if not self.publish_logged:
            self.get_logger().info("Publishing Combined Scan data")
            self.publish_logged = True


def main(args=None):
    rclpy.init(args=args)
    node = DataFuse()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
