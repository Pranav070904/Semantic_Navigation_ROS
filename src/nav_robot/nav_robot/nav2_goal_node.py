import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geographic_msgs.msg import GeoPoseStamped
import yaml
import os

from ament_index_python.packages import get_package_share_directory

class Nav2GoalNode(Node):
    def __init__(self):
        super().__init__('nav2_goal_node')

        try:
            pkg_share = get_package_share_directory('nav_robot')
            rooms_file = os.path.join(pkg_share, 'config', 'rooms.yaml')
        except Exception:
            rooms_file = os.path.abspath(
                os.path.join(os.path.dirname(__file__), '..', 'config', 'rooms.yaml'))

        with open(rooms_file,'r') as f:
            data = yaml.safe_load(f)

        self.rooms = data['rooms']


        self.get_logger().info(f'Loaded rooms: {list(self.rooms.keys())}')


        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # Subscribe to room goals from orchestrator
        self.nav_goal_sub = self.create_subscription(
            String, '/nav_goal', self.nav_goal_callback, 10)

        # Publish arrival back to orchestrator
        self.arrival_pub = self.create_publisher(
            String, '/nav_arrival', 10)

        self.current_room = None
        self.get_logger().info('Nav2 goal node ready')




    def nav_goal_callback(self,msg):
        room_name = msg.data.lower().strip().replace(' ','_')


        if room_name not in self.rooms:
            self.get_logger().info(
                f'Uknown room: {room_name}. Available: {list(self.rooms.keys())}'

            )
            return

        coords = self.rooms[room_name]
        self.current_room = room_name

        orientation_z = coords.get('orientation_z',0.0)
        orientation_w = coords.get('orientation_w',1.0)
        self.get_logger().info(
            f"Navigating to {room_name} at {(coords['x'],coords['y'])}"
        )

        self.send_goal(coords['x'],coords['y'],orientation_z,orientation_w)



    def send_goal(self,x,y,orientation_z=0.0,orientation_w=1.0):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(orientation_z)
        goal_msg.pose.pose.orientation.w = float(orientation_w)


        self.get_logger().info('Sending goal to Nav2...')

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self,feedback_msg):
        pass


    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted, navigating...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self,future):
        result = future.result()
        status = result.status



        if status == 4:
            self.get_logger().info(f'Arrived at {self.current_room}')
            arrival_msg = String()
            arrival_msg.data = self.current_room
            self.arrival_pub.publish(arrival_msg)
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')


def main(args = None):
    rclpy.init(args = args)
    node = Nav2GoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()