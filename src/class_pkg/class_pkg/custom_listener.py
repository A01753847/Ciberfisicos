import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

class CustomListener(Node):
    def __init__(self):
        super().__init__("custom_listener")
        self.get_logger().info("Custom listener is working.")
        self.subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.listener_callback, 10)
        
        self.i = 0

        self.client_TA = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        #service = TeleportAbsolute()
        self.initialize_pose()

    def initialize_pose(self):
        while not self.client_TA.wait_for_service(3):
            self.get_logger().info("Waiting for service...")

        request = TeleportAbsolute.Request()
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.0
        future = self.client_TA.call_async(request)
        future.add_done_callback(self.response_callback)
        
    def listener_callback(self, msg):
        self.i += 1 
        if self.i > 62:
            self.get_logger().info(f"Received: linear.x = {msg.x} , angular.z = {msg.y} ")
            self.i = 0

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info("Response", response)

def main(args=None):
    rclpy.init(args=args)
    node = CustomListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

