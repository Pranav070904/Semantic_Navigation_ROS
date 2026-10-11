import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from collections import deque
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
import os
import time
import math

class FrontierSearchNode(LifecycleNode):
    def __init__(self):
        super().__init__('frontier_search_node')
        self.get_logger().info("Frontier search node initialised")
        self.height = None
        self.width = None
        self.latest_map = None
        self.active = False
        self.margin = 0.3

        # matches nav2's own navigability rule: cost < 253 (inscribed) for robot_radius
        # 0.35. Stricter values starve the frontier search - inflation_radius 0.7 means
        # a zero-cost window demands over a metre of clearance from every wall.
        self.safe_cost_threshold = 200
        self.safe_check_radius = 2
        self.progress_ref = None
        self.progress_time = 0.0

        self.room_bounds = None
        self.room_name = None
        self.search_target = None
        self.announced_arrival = False
        self.scan_requested = False
        self.centroid_attempted = False
        self.explorable_total = 0

        self.rotation_speed = 0.4
        self.rotation_duration = 3.9  # ~90 degrees per step
        self.max_rotation_steps = 4

        self.image_width = 640
        self.angular_gain = 0.002
        self.max_linear = 0.7
        self.max_angular = 0.4
        self.centre_thresh = 30

        self.camera_hfov = 1.047          # matches <horizontal_fov> in robot.urdf.xacro
        self.range_window = 2

        self.approach_stop_distance = 0.6
        self.approach_gain = 0.6
        self.approach_timeout = 25.0
        # below this, a range reading cannot be the object we can see
        self.min_object_width = 0.10
        self.approach_started = 0.0
        self.latest_scan = None
        self.scan_sub = None
        # must exceed the gap between dino detections (~1.4s at process_every_n=3 under
        # current sim load) or APPROACHING expires between every frame and never moves
        self.box_timeout = 4.0
        self.last_box_time = 0.0
        self.last_pause_time = 0.0
        self.pause_timeout = 1.5  # if CLIP rejects, no /detected_box ever follows the pause
        self.max_hold = 4.0       # absolute cap, regardless of how often dino re-pauses
        self.hold_started = 0.0
        self.pause_sub = None

        self.box_sub = None
        self.arrival_pub = None
        self.scan_request_pub = None
        self.found_pub = None
        self.room_sub = None
        self.search_target_sub = None
        self.map_subscriber = None
        self.costmap_client = None
        self.costmap_timer = None
        self.costmap_pending = False
        self.nav_client = None
        self.marker_pub = None
        
        
        self.robot_x = None
        self.robot_y = None
        self.visited_grid = None
        self.current_goal_handle = None
        self.latest_costmap = None


        self.control_timer = None
        self.state_name = None
        self.cmd_pub = None

        self.failed_targets = []
        self.failed_target_radius = 0.4
        self.last_target = None
        self.same_target_count = 0

        rooms_path = os.path.join(
            get_package_share_directory('nav_robot'), 'config', 'rooms.yaml')
        with open(rooms_path, 'r') as f:
            self.all_rooms = yaml.safe_load(f)['rooms']

        self.no_target_cycles = 0


################################# LIFECYCLE FUNCTION ###################################################



    def on_configure(self, state:State) -> TransitionCallbackReturn:
        map_qos = QoSProfile(
            depth = 1,
            durability =DurabilityPolicy.TRANSIENT_LOCAL,
            reliability =ReliabilityPolicy.RELIABLE
        )
        self.marker_pub = self.create_publisher(MarkerArray,'/frontier_markers',10)
        self.map_subscriber = self.create_subscription(OccupancyGrid,'/map',self.map_callback,map_qos)
        # AMCL only publishes /amcl_pose after the robot moves update_min_d/_a, so waiting on
        # it deadlocks: this node is what starts the motion. map->base_link ticks regardless.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self,NavigateToPose,'navigate_to_pose')
        # The /global_costmap/costmap topic is unusable on Jazzy nav2 1.3.12: Costmap2DPublisher
        # never emits a grid to late-joining subscribers (ros-navigation/navigation2#6267).
        # The service returns the same grid, so poll it instead.
        self.costmap_client = self.create_client(GetCostmap,'/global_costmap/get_costmap')
        self.costmap_debug_pub = self.create_publisher(
            OccupancyGrid,'/frontier_costmap_debug',map_qos)

        self.room_sub = self.create_subscription(String,'/nav_goal',self.room_callback,10)
        self.search_target_sub = self.create_subscription(
            String,'/search_target',self.search_target_callback,10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arrival_pub = self.create_publisher(String,'/nav_arrival',10)
        self.scan_request_pub = self.create_publisher(String,'/scan_request',10)
        self.found_pub = self.create_publisher(String,'/target_found',10)
        self.box_sub = self.create_subscription(Twist,'/detected_box',self.box_callback,10)
        self.scan_sub = self.create_subscription(
            LaserScan,'/scan',self.scan_callback,rclpy.qos.qos_profile_sensor_data)
        self.pause_sub = self.create_subscription(Empty,'/detection_pause',self.pause_callback,10)
        self.get_logger().info("Frontier node Configured")
        return TransitionCallbackReturn.SUCCESS




    def on_activate(self, state):
        self.failed_targets, self.last_target = [], None
        self.same_target_count, self.no_target_cycles = 0, 0
        self.progress_ref = None
        self.announced_arrival = False
        self.scan_requested = False
        self.centroid_attempted = False
        self.active = True
        self.state_name = 'WAITING'
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.costmap_timer = self.create_timer(1.0, self.request_costmap)
        self.request_costmap()
        return TransitionCallbackReturn.SUCCESS






    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.active = False

        # cancel any in-flight Nav2 goal so it can't finish into the next room's run
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.stop_robot()

        if self.control_timer is not None:
            self.control_timer.cancel()
            self.destroy_timer(self.control_timer)
            self.control_timer = None

        if self.costmap_timer is not None:
            self.costmap_timer.cancel()
            self.destroy_timer(self.costmap_timer)
            self.costmap_timer = None
        self.costmap_pending = False

        # room_name/search_target must go too, or re-entering the same room or re-issuing
        # the same target later looks like a no-op resend
        self.room_bounds = None
        self.room_name = None
        self.search_target = None
        self.visited_grid = None
        self.explorable_total = 0
        self.state_name = None
        self.get_logger().info("Frontier Node deactivated")
        return TransitionCallbackReturn.SUCCESS




    

    def map_callback(self,msg):
        self.latest_map = msg
        self.height = msg.info.height
        self.width = msg.info.width
        
        self.get_logger().info(
            f'Map Received: {self.width} x {self.height}'
            f'Resolution: {msg.info.resolution}',
            throttle_duration_sec = 5.0
        )
        

        
    
    def get_visited_cells(self):
        visited_cells = []
        for r in range(self.height):
            for c in range(self.width):
                if self.visited_grid[self.to_index(r, c)]:
                    visited_cells.append((r, c))
        return visited_cells

    def get_unvisited_cells(self):
        unvisited_cells = []
        for r in range(self.r_lo,self.r_hi + 1):
            for c in range(self.c_lo,self.c_hi + 1):
                if not self.visited_grid[self.to_index(r, c)]:
                    unvisited_cells.append((r, c))
        return unvisited_cells



########################################### HELPER FUNCTION #######################################################################
    def compute_scan_window(self):
        b = self.room_bounds
        r0, c0 = self.world_to_grid(b['min_x'], b['min_y'])
        r1, c1 = self.world_to_grid(b['max_x'], b['max_y'])
        self.r_lo = max(0, min(r0, r1) - 2)
        self.r_hi = min(self.height - 1, max(r0, r1) + 2)
        self.c_lo = max(0, min(c0, c1) - 2)
        self.c_hi = min(self.width - 1, max(c0, c1) + 2)


    def is_recently_failed(self,x,y):
        for fx,fy in self.failed_targets:
            if math.sqrt((x-fx)**2 + (y-fy) **2) < self.failed_target_radius:
                return True
        return False
    
    def to_index(self,row,col):
        return row * self.width + col

    def world_to_grid(self,world_x,world_y):
            col = int((world_x - self.latest_map.info.origin.position.x)/self.latest_map.info.resolution)
            row = int((world_y - self.latest_map.info.origin.position.y)/self.latest_map.info.resolution)
            return row,col
    
    def grid_to_world(self,row,col):
        world_x = self.latest_map.info.origin.position.x + (col * self.latest_map.info.resolution)
        world_y = self.latest_map.info.origin.position.y + (row * self.latest_map.info.resolution)
        return world_x,world_y
    
    def get_cell(self,row,col):
        if row < 0 or row>=self.height or col < 0 or col >= self.width:
            return None
        return self.latest_map.data[self.to_index(row,col)]

    def mark_visited(self,world_x,world_y,radius = 5):
        x,y = self.world_to_grid(world_x,world_y)

        for dr in range(-radius,radius + 1):
            for dc in range(-radius,radius + 1):
                r = x + dr
                c = y + dc

                if 0<= r < self.height and 0 <= c < self.width:
                    self.visited_grid[self.to_index(r,c)] = True
        

    def control_loop(self):
        self.update_pose()
        if self.state_name == 'APPROACHING':
            if time.time() - self.last_box_time > self.box_timeout:
                self.get_logger().info('Lost the box - resuming frontier search')
                self.stop_robot()
                self.state_name = 'PLANNING'
            return

        if self.state_name == 'HOLDING':
            # dino re-pauses every frame it still sees the candidate, refreshing
            # last_pause_time faster than pause_timeout expires - without the absolute
            # cap a rejected false positive pins us here forever.
            if (time.time() - self.last_pause_time > self.pause_timeout
                    or time.time() - self.hold_started > self.max_hold):
                self.get_logger().info('Detection pause expired - resuming frontier search')
                self.state_name = 'PLANNING'
            return

        if self.state_name == 'WAITING':
            missing = [n for n, ok in [('map', self.latest_map is not None),
                                    ('costmap', self.latest_costmap is not None),
                                    ('pose', self.robot_x is not None),
                                    ('room', self.room_bounds is not None)] if not ok]
            if missing:
                self.get_logger().info(f'Waiting for: {missing}', throttle_duration_sec=2.0)
                return
            self.initialize_visited_grid()
            self.get_logger().info(f'Initial coverage: {self.get_coverage_percentage():.1f}%')
            self.state_name = 'PLANNING'
        elif self.state_name == 'PLANNING':
            self.plan_next_frontier()
        elif self.state_name == 'TRAVELING':
            self.check_progress()
        elif self.state_name == 'DONE':
            self.request_scene_scan()

    def request_scene_scan(self):
        if self.scan_requested:
            return
        self.scan_requested = True
        self.stop_robot()
        self.get_logger().info('Room exploration exhausted - requesting scene scan')
        msg = String()
        msg.data = 'scan_now'
        self.scan_request_pub.publish(msg)
    

    def check_progress(self):
        if self.progress_ref is None:
            return
        moved = math.hypot(self.robot_x - self.progress_ref[0],
                        self.robot_y - self.progress_ref[1])
        if moved > 0.15:
            self.progress_ref = (self.robot_x, self.robot_y)
            self.progress_time = time.time()
        elif time.time() - self.progress_time > 10.0:
            self.get_logger().warn('No progress for 10 s - canceling and blacklisting target')
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            if self.last_target is not None:
                self.failed_targets.append(self.last_target)
            self.state_name = 'PLANNING'

    def publish_frontier_markers(self, cells, ns='frontiers', color=(1.0, 0.0, 0.0), marker_id_offset=0):
        marker_array = MarkerArray()
        
        for i, (row, col) in enumerate(cells):
            world_x, world_y = self.grid_to_world(row, col)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = i + marker_id_offset
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)







    def initialize_visited_grid(self):
        if self.visited_grid is not None:   # resuming the same room, keep coverage
            return
        self.compute_scan_window()
        self.visited_grid = [True] * (self.width * self.height)   # default: excluded
        n_notfree = n_close = n_open = 0

        for r in range(self.r_lo, self.r_hi + 1):
            for c in range(self.c_lo, self.c_hi + 1):
                wx, wy = self.grid_to_world(r, c)

                if not (self.room_bounds['min_x'] <= wx <= self.room_bounds['max_x'] and
                        self.room_bounds['min_y'] <= wy <= self.room_bounds['max_y']):
                    continue
                if self.get_cell(r, c) != 0:
                    n_notfree += 1
                    continue
                if not self.is_point_safe(wx, wy, cost_threshold=self.safe_cost_threshold,
                                          check_radius=self.safe_check_radius):
                    n_close += 1
                    continue

                self.visited_grid[self.to_index(r, c)] = False        # searchable
                n_open += 1

        # measured against what we can actually reach - counting the not-free/too-close
        # cells starts coverage at ~84% before moving an inch
        self.explorable_total = n_open

        self.get_logger().info(
            f'init: window=({self.r_lo}-{self.r_hi}, {self.c_lo}-{self.c_hi}) '
            f'not_free={n_notfree} too_close={n_close} open={n_open} '
            f'costmap_ready={self.latest_costmap is not None}')






    def publish_unvisited_debug(self):
        
        unvisited_cells = []
        for r in range(self.height):
            for c in range(self.width):
                if not self.visited_grid[self.to_index(r, c)]:
                    unvisited_cells.append((r, c))
        self.publish_frontier_markers(unvisited_cells)






    def get_region_centroid(self,regions):
        avg_row = sum(r for r,_ in regions)/len(regions)
        avg_col = sum(c for _,c in regions)/len(regions)
        return avg_row,avg_col


    def rotation_step_logic(self):
        elapsed = time.time() - self.rotation_start_time
        
        if self.rotation_step < self.max_rotation_steps:
            if elapsed < self.rotation_duration:
                cmd = Twist()
                cmd.angular.z = self.rotation_speed
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.rotation_step += 1
                self.rotation_start_time = time.time()
        else:
            # full rotation done, no detection interrupted it
            self.stop_robot()
            self.mark_visited(self.robot_x, self.robot_y, radius=10)
            self.state_name = 'PLANNING'  


            

    def stop_robot(self):
        if self.cmd_pub is not None:
            self.cmd_pub.publish(Twist())

    def get_coverage_percentage(self):
        """Fraction of the cells that were REACHABLE at init which we have since visited.
        Cells excluded at init (wall, unknown, too close to an obstacle) are not in the
        denominator - they can never be driven to, so counting them would report the room
        as mostly covered before exploring any of it."""
        if not self.explorable_total:
            return 100.0

        still_open = 0
        for r in range(self.r_lo, self.r_hi + 1):
            for c in range(self.c_lo, self.c_hi + 1):
                if not self.visited_grid[self.to_index(r, c)]:
                    still_open += 1

        visited = self.explorable_total - still_open
        return (visited / self.explorable_total) * 100.0



##################################### FRONTIER FUNCTIONS ################################################################
    

    
    def cluster_frontiers(self,frontiers):
        frontier_set = set(frontiers)
        clustered = set()
        regions = []

        for cell in frontiers:
            if cell in clustered:
                continue

            region = []
            queue = deque([cell])
            clustered.add(cell)


            while queue:
                current = queue.popleft()
                region.append(current)
                r,c = current

                for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nei = (r + dr,c + dc)
                    if nei in frontier_set and nei not in clustered:
                        clustered.add(nei)
                        queue.append(nei)

            regions.append(region)


        return regions



    def find_frontier(self):
        frontiers = []
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(-1,-1),(-1,1),(1,-1),(1,1)]
        for r in range(self.r_lo,self.r_hi+1):
            for c in range(self.c_lo,self.c_hi+1):
                index = self.to_index(r,c)

                if(self.visited_grid[index] == False):
                    for dx,dy in dirs:
                        newr = r + dx
                        newc = c + dy
                        if 0 <= newr < self.height and 0 <= newc < self.width:
                            new_idx = self.to_index(newr,newc)
                            if(self.visited_grid[new_idx] == True):
                                frontiers.append((r,c))
                                break

        return frontiers




    def score_region(self, region):
        cand = []
        for (r, c) in region:
            wx, wy = self.grid_to_world(r, c)
            cand.append((math.hypot(wx - self.robot_x, wy - self.robot_y), r, c, wx, wy))
        cand.sort()

        for d, r, c, wx, wy in cand:
            if d < 0.15 or self.is_recently_failed(wx, wy):
                continue
            # same strict check as init, against the CURRENT costmap
            if not self.is_point_safe(wx, wy, cost_threshold=self.safe_cost_threshold,
                                      check_radius=self.safe_check_radius):
                self.visited_grid[self.to_index(r, c)] = True   # exclude it for good
                continue
            return d, (wx, wy), len(region)
        return float('inf'), None, len(region)

    def pick_best_frontier(self, regions):
        best_dist, best_region, best_xy = float('inf'), None, None
        for region in regions:
            dist, point, size = self.score_region(region)
            if point is not None and dist < best_dist:
                best_dist, best_region, best_xy = dist, region, point
        return best_region, best_xy
        

    def send_frontier_goal(self, x, y):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.state_name = 'PLANNING'
            return

        dx = x - self.robot_x
        dy = y - self.robot_y
        yaw = math.atan2(dy, dx)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending frontier goal: ({x:.2f},{y:.2f})')

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)



    def finish_exploration(self):
        """Nothing left to explore. If we never actually reached this room (a cluttered or
        already-mapped room can have no reachable frontiers at all), drive to its centre
        first - otherwise we'd report 'arrived' from wherever we happen to be standing."""
        if self.announced_arrival or self.centroid_attempted:
            self.state_name = 'DONE'
            return

        self.centroid_attempted = True
        b = self.room_bounds
        cx = (b['min_x'] + b['max_x']) / 2.0
        cy = (b['min_y'] + b['max_y']) / 2.0
        self.get_logger().info(
            f'No frontiers but never entered {self.room_name} - driving to its centre')
        self.state_name = 'TRAVELING'
        self.progress_ref = (self.robot_x, self.robot_y)
        self.progress_time = time.time()
        self.send_frontier_goal(cx, cy)

    def plan_next_frontier(self):
        if self.robot_x is None:
            self.get_logger().warn('No pose yet - waiting', throttle_duration_sec=2.0)
            return

        coverage = self.get_coverage_percentage()
        self.get_logger().info(f'Coverage: {coverage:.1f}%', throttle_duration_sec=2.0)
        if coverage >= 90.0:
            self.get_logger().info('Coverage threshold reached - search exhausted')
            self.finish_exploration()
            return

        frontier = self.find_frontier()
        regions = self.cluster_frontiers(frontier)

        self.publish_frontier_markers(self.get_unvisited_cells(), ns='unvisited', color=(0.0, 0.0, 1.0))
        self.publish_frontier_markers(frontier, ns='frontier_boundary', color=(1.0, 0.0, 0.0))

        if not regions:
            self.get_logger().info('No frontiers left - search exhausted')
            self.finish_exploration()
            return

        best_region, best_xy = self.pick_best_frontier(regions)

        if best_xy is None:
            self.no_target_cycles += 1
            if self.no_target_cycles >= 20:           # ~2 s with nothing reachable
                self.get_logger().warn('No reachable frontier left - ending search')
                self.finish_exploration()
            return
        self.no_target_cycles = 0

        # repeated same target means we are stuck in a success/fail loop
        if self.last_target is not None:
            d = math.hypot(best_xy[0] - self.last_target[0], best_xy[1] - self.last_target[1])
            self.same_target_count = self.same_target_count + 1 if d < 0.2 else 0

        if self.same_target_count >= 2:
            self.get_logger().warn(f'Same target repeated - blacklisting {best_xy}')
            self.failed_targets.append(best_xy)
            self.same_target_count = 0
            self.last_target = None
            return

        self.last_target = best_xy
        distance = math.hypot(best_xy[0] - self.robot_x, best_xy[1] - self.robot_y)
        self.get_logger().info(f'Next target: {best_xy}, distance: {distance:.2f}m')

        self.progress_ref = (self.robot_x, self.robot_y)
        self.progress_time = time.time()
        self.state_name = 'TRAVELING'
        self.send_frontier_goal(best_xy[0], best_xy[1])


    # cost_threshold is on the raw 0-255 costmap scale, not the 0-100 scale the
    # OccupancyGrid topic used: 254 is lethal, 253 inscribed, 255 unknown.
    def is_point_safe(self, world_x, world_y, cost_threshold=64, check_radius=2):
        if self.latest_costmap is None:
            return True

        md = self.latest_costmap.metadata
        col = int((world_x - md.origin.position.x) / md.resolution)
        row = int((world_y - md.origin.position.y) / md.resolution)
        w = md.size_x
        h = md.size_y

        for dr in range(-check_radius, check_radius + 1):
            for dc in range(-check_radius, check_radius + 1):
                r, c = row + dr, col + dc
                if r < 0 or r >= h or c < 0 or c >= w:
                    return False
                if self.latest_costmap.data[r * w + c] >= cost_threshold:
                    return False
        return True



######################################### CALLBACKS ##########################################################
    def goal_response_callback(self, future):
        goal_handle = future.result()

        # node was deactivated while the goal was being sent: cancel it and bail
        if not self.active:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Frontier rejected - blacklisting and replanning')
            if self.last_target is not None:
                self.failed_targets.append(self.last_target)
            self.state_name = 'PLANNING'
            return
        if self.state_name != 'TRAVELING':
            goal_handle.cancel_goal_async()
            return
        self.current_goal_handle = goal_handle
        self.get_logger().info('Frontier goal accepted, navigating..')
        goal_handle.get_result_async().add_done_callback(
            lambda f, h=goal_handle: self.goal_result_callback(f, h))

    def goal_result_callback(self, future, handle):
        # ignore results from goals that were canceled, replaced, or finished
        # after the node was deactivated
        if not self.active or handle is not self.current_goal_handle:
            return
        self.current_goal_handle = None

        result = future.result()
        status = result.status
        self.get_logger().info(f'Goal result status: {status}')

        if status != 4:   # 4 = SUCCEEDED
            self.get_logger().warn(f'Navigation failed (status {status}) - blacklisting')
            if self.last_target is not None:
                self.failed_targets.append(self.last_target)
            self.state_name = 'PLANNING'
            return

        self.get_logger().info('Arrived at frontier - marking visited')
        self.mark_visited(self.robot_x, self.robot_y, radius=10)

        self.announce_arrival()
        self.state_name = 'PLANNING'

    
    # orch gates grounding_dino on this: it stays down until we are actually in the room
    def announce_arrival(self):
        if self.announced_arrival:
            return
        self.announced_arrival = True
        arrival = String()
        arrival.data = self.room_name or ''
        self.arrival_pub.publish(arrival)
        self.get_logger().info(f'Arrived in {self.room_name} - announcing arrival')

    def pause_callback(self, msg):
        if not self.active or self.state_name in (None, 'WAITING', 'APPROACHING'):
            return
        self.last_pause_time = time.time()
        if self.state_name == 'HOLDING':
            return
        self.get_logger().info('Detection candidate seen - holding for CLIP verification')
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.stop_robot()
        self.hold_started = time.time()
        self.state_name = 'HOLDING'

    def box_callback(self, msg):
        if not self.active or self.state_name in (None, 'WAITING'):
            return
        self.last_box_time = time.time()

        if self.state_name != 'APPROACHING':
            # Nav2 owns /cmd_vel while TRAVELING, so its goal must go before we drive manually
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.get_logger().info('Target spotted - approaching')
            self.approach_started = time.time()
            self.state_name = 'APPROACHING'

        self.approach_step(msg)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def range_to_box(self, box_cx):
        """Actual distance to whatever is under the detected box, from the lidar. Box area
        can't be used for this: a small or partly occluded object never grows enough to
        cross an area threshold no matter how close we get."""
        scan = self.latest_scan
        if scan is None or not scan.ranges or not scan.angle_increment:
            return None

        # pixel offset from image centre -> bearing. +x in the image is to the robot's
        # right, which is negative yaw.
        bearing = -(box_cx - self.image_width / 2.0) / self.image_width * self.camera_hfov
        centre = int(round((bearing - scan.angle_min) / scan.angle_increment))

        best = None
        for i in range(centre - self.range_window, centre + self.range_window + 1):
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                    best = r if best is None else min(best, r)
        return best

    def lidar_matches_box(self, dist, box_w):
        """Is this range reading plausibly the object we can see? A box spanning box_w
        pixels at distance dist implies a real-world width; if that comes out absurdly
        small the beam found something closer than the target, not the target itself."""
        if not box_w:
            return True        # no width info, nothing to check against
        frac = box_w / self.image_width
        implied_width = 2.0 * dist * math.tan(frac * self.camera_hfov / 2.0)
        return implied_width >= self.min_object_width

    def report_target_reached(self, why):
        self.stop_robot()
        self.state_name = 'DONE'
        self.scan_requested = True      # arrived at the target, no scene scan wanted
        found = String()
        found.data = 'target reached'
        self.found_pub.publish(found)
        self.get_logger().info(f'Target reached ({why}) - reporting found')

    def approach_step(self, msg):
        box_cx = msg.linear.x
        box_w = msg.linear.z
        error_x = box_cx - self.image_width / 2
        dist = self.range_to_box(box_cx)

        if dist is not None and dist <= self.approach_stop_distance:
            if self.lidar_matches_box(dist, box_w):
                self.report_target_reached(f'{dist:.2f}m away')
                return
            
            self.get_logger().info(
                f'Ignoring {dist:.2f}m lidar hit - too close for a box this small',
                throttle_duration_sec=3.0)
            dist = None

        if time.time() - self.approach_started > self.approach_timeout:
            self.report_target_reached('approach timed out')
            return

        cmd = Twist()
        if abs(error_x) > self.centre_thresh:
            cmd.angular.z = max(-self.max_angular,
                                min(self.max_angular, -self.angular_gain * error_x))
        else:
            if dist is None:
                cmd.linear.x = 0.15        # creep in blind
            else:
                cmd.linear.x = max(0.08, min(
                    self.max_linear,
                    self.approach_gain * (dist - self.approach_stop_distance)))
        self.cmd_pub.publish(cmd)

    def update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'No map->base_link yet: {e}', throttle_duration_sec=5.0)
            return
        self.robot_x = t.transform.translation.x
        self.robot_y = t.transform.translation.y


    def request_costmap(self):
        if self.costmap_pending:
            return
        if not self.costmap_client.service_is_ready():
            self.get_logger().warn('Costmap service not ready', throttle_duration_sec=5.0)
            return
        self.costmap_pending = True
        self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(
            self.costmap_response)

    def costmap_response(self, future):
        self.costmap_pending = False
        grid = future.result().map
        if self.latest_costmap is None:
            self.get_logger().info(
                f'First costmap received: {grid.metadata.size_x}x{grid.metadata.size_y}')
        self.latest_costmap = grid
        self.republish_costmap(grid)

    def republish_costmap(self, grid):
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = grid.metadata.resolution
        out.info.width = grid.metadata.size_x
        out.info.height = grid.metadata.size_y
        out.info.origin = grid.metadata.origin
        out.data = [-1 if v == 255 else min(100, v * 100 // 254) for v in grid.data]
        self.costmap_debug_pub.publish(out)

    def room_callback(self, msg):
        name = msg.data.lower().strip().replace(' ', '_')
        # orch republishes the room on every SEARCH_FOR, which must not wipe progress
        if name == self.room_name:
            self.get_logger().info(f'Already searching {name} - keeping progress')
            return
        room = self.all_rooms.get(name)
        if room is None or 'min_x' not in room:
            self.get_logger().warn(f'No bounds for room "{msg.data}"')
            return
        x0, x1 = sorted((room['min_x'], room['max_x']))
        y0, y1 = sorted((room['min_y'], room['max_y']))
        self.room_bounds = {
            'min_x': x0 + self.margin, 'max_x': x1 - self.margin,
            'min_y': y0 + self.margin, 'max_y': y1 - self.margin,
        }
        self.room_name = name
        self.reset_search()
        self.get_logger().info(f'Room set to {name} | Bounds: {self.room_bounds}')

    def search_target_callback(self, msg):
        target = msg.data.strip().lower()
        is_new = target != self.search_target
        self.search_target = target

        if self.state_name != 'DONE':
            # still exploring - dino just retargets, our coverage stays valid
            self.scan_requested = False
            return

        # parked in DONE: something must happen or orch waits in SEARCHING forever for
        # a /scene_objects only we can trigger
        if is_new:
            # swept with dino hunting a different object, so that coverage is stale here
            self.get_logger().info(f'New search target "{target}" - re-arming exhausted room')
            self.reset_search()
        else:
            # already swept for this one; re-driving would just loop
            self.get_logger().info(
                f'Target "{target}" already swept here - re-requesting scene scan')
            self.scan_requested = False

    def reset_search(self):
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.stop_robot()
        self.visited_grid = None
        self.explorable_total = 0
        self.failed_targets, self.last_target = [], None
        self.same_target_count, self.no_target_cycles = 0, 0
        self.progress_ref = None
        self.announced_arrival = False
        self.scan_requested = False
        self.centroid_attempted = False
        self.state_name = 'WAITING'

def main(args = None):
    rclpy.init(args=args)
    node = FrontierSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()