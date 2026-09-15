import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from lifecycle_msgs.srv import ChangeState,GetState
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import GetState
import json




class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('orch_node')



        self.state = 'IDLE'
        self.goal = None
        self.last_searched = None
        self.current_room = None
        self.visited_rooms = []
        self.searched_items = []

        self.perception_active = False

        self.create_subscription(String,'/goal',self.goal_callback,10)
        self.create_subscription(String,'/reasoning_action',self.action_callback,10)
        self.create_subscription(String,'/scene_objects',self.scene_callback,10)
        self.create_subscription(String,'/nav_arrival',self.arrival_callback,10)
        self.create_subscription(String,'/target_found',self.found_callback,10)


        self.reasoning_feedback_pub = self.create_publisher(String,'/reasoning_feedback',10)
        self.search_target_pub = self.create_publisher(String, '/search_target', 10)
        self.nav_goal_pub = self.create_publisher(String, '/nav_goal', 10)

        self.dino_change_state = self.create_client(
            ChangeState,'/grounding_dino_node/change_state'
        )

        self.search_change_state = self.create_client(ChangeState,'/local_search_node/change_state')

        self.get_logger().info('Orchestrator ready. Configuring Perception stack')
        self._configure_perception_on_startup()


    def _call_transition(self,client,transition_id,name):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        future.add_done_callback(
            lambda f:self._transition_done(f,name,transition_id)
        )

    def _configure_perception_on_startup(self):
        "Called once on boot"


        self._wait_for_service(self.dino_change_state,'grounding_dino_node')
        self._wait_for_service(self.search_change_state,'local_search_node')

        self._call_transition(self.dino_change_state, Transition.TRANSITION_CONFIGURE,'grounding_dino_node')
        self._call_transition(self.search_change_state,Transition.TRANSITION_CONFIGURE,'local_search_node')

    def _wait_for_service(self,client,name,timout=15.0):
        if not client.wait_for_service(timeout_sec = timout):
            self.get_logger().error(f'{name} lifecycle service is not available.')


    def _transition_done(self, future, name, transition_id):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'{name}: transition {transition_id} succeeded')
            else:
                self.get_logger().error(f'{name}: transition {transition_id} FAILED')
        except Exception as e:
            self.get_logger().error(f'{name}: transition {transition_id} exception: {e}')


    def activate_perception(self):
        if self.perception_active:
            return
        self.get_logger().info("Activating perception stack")
        self._call_transition(self.dino_change_state, Transition.TRANSITION_CONFIGURE, 'grounding_dino_node')
        self._call_transition(self.search_change_state, Transition.TRANSITION_CONFIGURE, 'local_search_node')
        self._call_transition(self.dino_change_state, Transition.TRANSITION_ACTIVATE, 'grounding_dino_node')
        self._call_transition(self.search_change_state, Transition.TRANSITION_ACTIVATE, 'local_search_node')
        self.perception_active = True

    def deactivate_perception(self):
        if not self.perception_active:
            return
        self.get_logger().info('Deactivating perception stack')
        self._call_transition(self.dino_change_state,Transition.TRANSITION_DEACTIVATE,'grounding_dino_node')
        self._call_transition(self.search_change_state,Transition.TRANSITION_DEACTIVATE,'local_search_node')
        self.perception_active = False

    def goal_callback(self,msg):
        self.get_logger().info(f'GOAL CALLBACK ENTERED: {msg.data}')  # add this line first
        if self.state != 'IDLE':
            self.get_logger().info('Busy. Ignoring New Goal')
            return
        #self.deactivate_perception()


        self.goal = msg.data
        self.visited_rooms = []
        self.searched_items = []
        self.state = 'REASONING'
        self.get_logger().info(f'Goal: {self.goal} | State: Reasoning')
        trigger_msg = String()
        trigger_msg.data = f'NEW_GOAL: {self.goal}'
        self.reasoning_feedback_pub.publish(trigger_msg)


    def action_callback(self,msg):
        try:
            action = json.loads(msg.data)
            action_type = action['action']
            target = action.get('target') or ''
            reasoning = action.get('reasoning','')

            self.get_logger().info(f'{action_type} → {target} | {reasoning}')

            if action_type == 'GO_TO_ROOM':
                self.state = 'NAVIGATING'
                self.visited_rooms.append(target)
                nav_msg = String()
                nav_msg.data = target
                self.nav_goal_pub.publish(nav_msg)

            elif action_type == 'SEARCH_FOR':
                self.state = 'SEARCHING'
                self.last_searched = target
                
                if target != self.goal:
                    self.searched_items.append(target)

                search_msg = String()
                search_msg.data = target
                self.search_target_pub.publish(search_msg)
                self.activate_perception()

            elif action_type == 'TASK_COMPLETE':
                self.deactivate_perception()
                self.state = 'IDLE'
                self.get_logger().info('TASK COMPLETE')


            elif action_type == 'TASK_FAILED':
                self.deactivate_perception()
                self.state = 'IDLE'
                self.get_logger().info(f'TASK FAILED: {reasoning}')

        except Exception as e:
            self.get_logger().error(f'Action Parsing Failed: {e}')



    def arrival_callback(self, msg):
        if self.state != 'NAVIGATING':
            return

        if self.goal is None:
            self.get_logger().warn('Arrived, but no goal is set — ignoring stale arrival')
            self.state = 'IDLE'
            return

        self.current_room = msg.data
        self.searched_items = []
        self.last_searched = self.goal
        self.state = 'SEARCHING'
        self.get_logger().info(f'Arrived at {msg.data} | State: SEARCHING')

        search_msg = String()
        search_msg.data = self.goal
        self.search_target_pub.publish(search_msg)

        self.activate_perception()

    def scene_callback(self, msg):
        if self.state != 'SEARCHING':
            return

        self.deactivate_perception()
        self.state = 'REASONING'

        feedback = (
            f'I am in the {self.current_room}. '
            f'I searched for "{self.last_searched}" but could not find it. '
            f'Containers already checked in this room: {self.searched_items}. '
            f'I can currently see: {msg.data}. '
            f'Rooms already visited: {self.visited_rooms}. '
            f'What should I do next?'
        )
        self.get_logger().info(f'Feedback: {feedback}')
        feedback_msg = String()
        feedback_msg.data = feedback
        self.reasoning_feedback_pub.publish(feedback_msg)

    def found_callback(self, msg):
        self.deactivate_perception()
        self.state = 'IDLE'
        self.get_logger().info(f'TARGET FOUND: {msg.data}')

    

def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()