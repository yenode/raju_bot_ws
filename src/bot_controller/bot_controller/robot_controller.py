import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Controller has been started. Moving bot forward...')

    def timer_callback(self):
        msg = Twist()
        # Move forward
        msg.linear.x = 0.5
        # Add slight rotation just to see it clearly
        msg.angular.z = 0.2
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
        
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
