import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot Controller has been started. Moving bot forward...')

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        # Move forward
        msg.twist.linear.x = 0.5
        # Add slight rotation just to see it clearly
        msg.twist.angular.z = 0.2
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
