import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from irobot_create_msgs.action import Dock  

class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waypoint Planner Node Initialized")

        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)
        self.points = []
        self.get_logger().info("Waiting for two points...")

        self.battery_subscription = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)

        self._dock_client = ActionClient(self, Dock, 'dock')
        self._dock_client.wait_for_server()
        self.get_logger().info("Dock action server is available")

        self.current_target_index = 0
        self.battery_level = 100.0

    def point_callback(self, msg):
        self.get_logger().info(f"Received point: ({msg.point.x}, {msg.point.y})")
        self.points.append(msg)

        if len(self.points) == 2:
            self.get_logger().info(f"Received two points.")
            self.start_navigation_loop()

    def start_navigation_loop(self):
        if len(self.points) == 2:
            self.current_target_index = 0
            self.navigate_to_next_point()

    def navigate_to_next_point(self):
        if len(self.points) < 2:
            self.get_logger().error("Not enough points received.")
            return
        
        target_point = self.points[self.current_target_index]
        target_label = "Point A" if self.current_target_index == 0 else "Point B"
        
        self.get_logger().info(f"Preparing to navigate to {target_label}")
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_point.point.x
        goal_msg.pose.pose.position.y = target_point.point.y
        goal_msg.pose.pose.position.z = target_point.point.z
        goal_msg.pose.pose.orientation.w = 1.0 

        self.get_logger().info(f"Sending goal: {target_label}")
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info("Navigation to target successful")
            # Wait before sending the next goal
            self.create_timer(5.0, self.switch_target)
        else:
            self.get_logger().error("Navigation failed")

    def switch_target(self):
        self.current_target_index = 1 - self.current_target_index
        self.navigate_to_next_point()

    def battery_callback(self, msg):
        self.battery_level = msg.percentage
        self.get_logger().info(f"Current battery level: {self.battery_level}%")
        
        if self.battery_level < 95.0:  # Threshold for docking
            self.get_logger().warning("Battery low, docking...")
            self.perform_docking()

    def perform_docking(self):
        if not self._dock_client.action_server_is_ready():
            self.get_logger().error("Dock action server not available.")
            return

        goal_msg = Dock.Goal()
        future = self._dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.docking_response_callback)

    def docking_response_callback(self, future):
        try:
            result = future.result()
            if result:
                self.get_logger().info("Successfully docked!")
            else:
                self.get_logger().error("Docking failed.")
        except Exception as e:
            self.get_logger().error(f"Exception while calling docking action: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


