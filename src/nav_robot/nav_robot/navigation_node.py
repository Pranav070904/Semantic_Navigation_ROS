import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.image_width = 640
        self.image_height = 480


        self.angular_gain = .002
        self.linear_gain = .003
        self.max_linear = 0.8
        self.max_angular = 1.5
        self.target_area = 80000
        self.centre_thresh = 60


        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.box_sub = self.create_subscription(Twist,'detected_box',self.box_callback,10)

        self.last_detection_time = time.time()
        self.timeout = 1.0
        self.create_timer(0.1,self.safety_timer)

        self.get_logger().info('Navigation Node Ready')



    def box_callback(self,msg):
        self.last_detection_time = time.time()


        box_cx = msg.linear.x
        box_area = msg.linear.y


        image_cx = self.image_width/2
        error_x = box_cx - image_cx

        cmd = Twist()

        if box_area >=self.target_area:
            self.get_logger().info('Target Reached')
            self.cmd_pub.publish(cmd)
            return


        cmd.angular.z = -self.angular_gain * error_x
        cmd.angular.z = max(-self.max_angular,min(self.max_angular,cmd.angular.z))

        if(abs(error_x)<self.centre_thresh):
            cmd.linear.x = min(self.max_linear,self.linear_gain*(self.target_area-box_area))
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0


        self.get_logger().info(
            f'error_x: {error_x:.1f} area: {box_area:.0f} | lin: {cmd.linear.x:.2f} ang: {cmd.angular.z:.2f} '
        )

        self.cmd_pub.publish(cmd)


    def safety_timer(self):
        if time.time() - self.last_detection_time > self.timeout:
            self.cmd_pub.publish(Twist())


def main(args = None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()