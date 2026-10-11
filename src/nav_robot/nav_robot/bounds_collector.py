# bounds_collector.py - run with: python3 bounds_collector.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class Collector(Node):
    def __init__(self):
        super().__init__('bounds_collector')
        self.pts = []
        self.create_subscription(PointStamped, '/clicked_point', self.cb, 10)

    def cb(self, msg):
        self.pts.append((msg.point.x, msg.point.y))
        xs, ys = zip(*self.pts)
        print(f'{len(self.pts)} pts | min_x: {min(xs):.2f}  max_x: {max(xs):.2f}  '
              f'min_y: {min(ys):.2f}  max_y: {max(ys):.2f}')

rclpy.init()
n = Collector()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass