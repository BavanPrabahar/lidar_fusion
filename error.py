 import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import numpy as np

class TwoLaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger')
        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            LaserScan, "/scan1", self.first_callback, 1, callback_group=self.callback_group
        )
        self.create_subscription(
            LaserScan, "/scan2", self.second_callback, 1, callback_group=self.callback_group
        )

        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        self.f_msg = None
        self.s_msg = None
        self.ready_f = False
        self.ready_s = False

        self.x1 = []
        self.y1 = []
        self.x2 = []
        self.y2 = []

    def first_callback(self, msg):
        self.f_msg = msg
        self.process_first()
        self.ready_f = True
        self.try_merge()

    def second_callback(self, msg):
        self.s_msg = msg
        self.process_second()
        self.ready_s = True
        self.try_merge()

    def process_first(self):
        h = int((math.radians(135)) / self.f_msg.angle_increment)
        ranges = list(self.f_msg.ranges)

        s1 = ranges[:h]
        s2 = ranges[-h:]
        c = s1 + s2

        self.x1 = []
        self.y1 = []
        for i, r in enumerate(c):
            angle = i * self.f_msg.angle_increment
            self.x1.append(r * math.cos(angle) - 1)
            self.y1.append(r * math.sin(angle) + 1)

    def process_second(self):
        h = int((math.radians(135)) / self.s_msg.angle_increment)
        ranges = list(self.s_msg.ranges)

        s3 = ranges[:h]
        s4 = ranges[-h:]
        c = s3 + s4

        self.x2 = []
        self.y2 = []
        for i, r in enumerate(c):
            angle = i * self.s_msg.angle_increment
            self.x2.append(r * math.cos(angle) + 1)
            self.y2.append(r * math.sin(angle) - 1)

    def try_merge(self):
        if not (self.ready_f and self.ready_s):
            return

        self.ready_f = False
        self.ready_s = False

        merged_scan = LaserScan()
        merged_scan.header = self.f_msg.header 
        merged_scan.angle_min = self.f_msg.angle_min
        merged_scan.angle_max = self.f_msg.angle_max
        merged_scan.angle_increment = self.f_msg.angle_increment
        merged_scan.time_increment = self.f_msg.time_increment
        merged_scan.scan_time = self.f_msg.scan_time
        merged_scan.range_min = self.f_msg.range_min
        merged_scan.range_max = self.f_msg.range_max

        
        f = int(len(self.x1)) * 4 // 3  
        ranges = []

        for i in range(f):
            if i < f // 8 or i > f * 7 // 8:
                x = self.x1[i]
                y = self.y1[i]
            elif f // 8 < i < f * 3 // 8 or f * 5 // 8 < i < f * 7 // 8:
                x = (self.x1[i] + self.x2[i]) / 2
                y = (self.y1[i] + self.y2[i]) / 2
            elif f * 3 // 8 < i < f * 5 // 8:
                x = self.x2[i]
                y = self.y2[i]
            else:
                x = 0.0
                y = 0.0

            r = math.sqrt(x ** 2 + y ** 2)
            ranges.append(r)

        merged_scan.ranges = ranges
        self.pub.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = TwoLaserMerger()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
