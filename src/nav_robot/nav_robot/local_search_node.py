import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import math


class LocalSearchNode(LifecycleNode):
    def __init__(self):
        super().__init__('local_search_node')

        self.state = 'IDLE'  # IDLE, SEARCHING, APPROACHING
        self.search_step = 0
        self.max_search_steps = 4
        self.forward_cycles = 0
        self.max_forward_cycles = 2

        self.last_box_time = 0
        self.box_timeout = 1.0

        self.image_width = 640
        self.angular_gain = 0.002
        self.linear_gain = 0.0003
        self.max_linear = 0.7
        self.max_angular = 0.4
        self.target_area = 60000
        self.centre_thresh = 30

        # Fixed rotation math — 0.4 rad/s * 3.9s ≈ 1.57 rad ≈ 90°
        self.rotation_speed = 0.4
        self.rotation_duration = 3.9
        self.forward_duration = 1.5
        self.forward_speed = 0.3

        self.action_start_time = None

        self.cmd_pub = None
        self.box_sub = None
        self.search_target_sub = None
        self.found_sub = None
        self.scan_request_pub = None  # NEW
        self.control_timer = None

        self.get_logger().info('Local search node constructed (unconfigured)')

    # ---------------- LIFECYCLE ----------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.found_pub = self.create_publisher(String, '/target_found', 10)
        self.scan_request_pub = self.create_publisher(String, '/scan_request', 10)  # NEW

        self.search_target_sub = self.create_subscription(
            String, '/search_target', self.search_target_callback, 10)
        self.found_sub = self.create_subscription(
            String, '/target_found', self.found_callback, 10)

        self.get_logger().info('Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating')
        self.box_sub = self.create_subscription(
            Twist, '/detected_box', self.box_callback, 10)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating')
        self.stop_robot()
        self.state = 'IDLE'
        if self.box_sub is not None:
            self.destroy_subscription(self.box_sub)
            self.box_sub = None
        if self.control_timer is not None:
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)
            self.control_timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for sub in [self.box_sub, self.search_target_sub, self.found_sub]:
            if sub is not None:
                self.destroy_subscription(sub)
        self.box_sub = None
        self.search_target_sub = None
        self.found_sub = None
        for pub in [self.cmd_pub, self.found_pub, self.scan_request_pub]:
            if pub is not None:
                self.destroy_publisher(pub)
        self.cmd_pub = None
        self.found_pub = None
        self.scan_request_pub = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.stop_robot()
        return TransitionCallbackReturn.SUCCESS

    # ---------------- SEARCH / APPROACH ----------------

    def search_target_callback(self, msg):
        self.get_logger().info(f'Starting search for: {msg.data}')
        self.state = 'SEARCHING'
        self.search_step = 0
        self.forward_cycles = 0
        self.action_start_time = time.time()

    def box_callback(self, msg):
        self.last_box_time = time.time()
        if self.state == 'SEARCHING':
            self.get_logger().info('Target spotted during search — approaching')
            self.state = 'APPROACHING'
        if self.state == 'APPROACHING':
            self.approach_step(msg)

    def found_callback(self, msg):
        self.state = 'IDLE'
        self.stop_robot()

    def approach_step(self, msg):
        box_cx = msg.linear.x
        box_area = msg.linear.y
        image_cx = self.image_width / 2
        error_x = box_cx - image_cx

        cmd = Twist()
        self.get_logger().info(f'Approachning - area :{box_area:.2f}')

        if box_area >= self.target_area:
            self.stop_robot()
            self.state = 'IDLE'
            found_msg = String()
            found_msg.data = 'target reached'
            self.found_pub.publish(found_msg)
            self.get_logger().info('Target reached — reporting found')
            return

        if abs(error_x) > self.centre_thresh:
            cmd.angular.z = -self.angular_gain * error_x
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
            cmd.linear.x = 0.0
        else:
            cmd.angular.z = 0.0
            raw = self.linear_gain * (self.target_area - box_area)
            cmd.linear.x = max(0.08, min(self.max_linear, raw))

        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.state != 'SEARCHING':
            return

        if time.time() - self.last_box_time < self.box_timeout:
            return

        elapsed = time.time() - self.action_start_time

        if self.search_step < self.max_search_steps:
            if elapsed < self.rotation_duration:
                cmd = Twist()
                cmd.angular.z = self.rotation_speed
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.search_step += 1
                self.action_start_time = time.time()
                self.get_logger().info(
                    f'Search step {self.search_step}/{self.max_search_steps}')

        elif self.forward_cycles < self.max_forward_cycles:
            if elapsed < self.forward_duration:
                cmd = Twist()
                cmd.linear.x = self.forward_speed
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.forward_cycles += 1
                self.search_step = 0
                self.action_start_time = time.time()
                self.get_logger().info(
                    f'Moving forward, cycle {self.forward_cycles}/{self.max_forward_cycles}')

        else:
            # NEW — pattern genuinely exhausted, WE decide to give up,
            # not a race with Dino's own timer
            self.get_logger().info('Search pattern exhausted — requesting scene scan')
            self.stop_robot()
            self.state = 'IDLE'

            scan_request = String()
            scan_request.data = 'scan_now'
            self.scan_request_pub.publish(scan_request)

    def stop_robot(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LocalSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()