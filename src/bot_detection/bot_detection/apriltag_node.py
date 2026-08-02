import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

# Import the detector you wrote
from .apriltag_detector import AprilTagDetector

class PID:
    def __init__(self, kp, ki, kd, out_max, out_min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.out_min = out_min
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral += error * dt
        
        # Anti-windup (clamp integral)
        if self.ki != 0:
            int_max = self.out_max / self.ki
            int_min = self.out_min / self.ki
            self.integral = max(min(self.integral, int_max), int_min)
            
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.out_max), self.out_min)

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        self.bridge = CvBridge()
        
        # Initialize the detector
        self.detector = AprilTagDetector()
        
        # Subscribing to the simulated camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publisher for velocities
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for the debug visualization
        self.viz_pub = self.create_publisher(Image, '/camera/tag_visualization', 10)
        
        # PID Controllers
        # Linear: move forward if tag is further than safe_distance
        self.linear_pid = PID(kp=0.6, ki=0.05, kd=0.1, out_max=0.5, out_min=-0.5)
        # Angular: rotate to keep tag in center of image
        self.angular_pid = PID(kp=0.003, ki=0.0005, kd=0.001, out_max=1.0, out_min=-1.0)
        
        self.safe_distance = 1.0 # Stop exactly 1.0 meters away
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('AprilTag Detector Node with PID started!')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        h, w = cv_image.shape[:2]
        
        # Run detection
        tags = self.detector.detect_tags(cv_image, rgb_input=False)
        tag_data = self.detector.find_target_tag(tags, w, h)
        
        # Draw and publish visualization
        viz_image = self.detector.draw_visualization(cv_image, tag_data)
        if viz_image is not None:
            viz_msg = Image()
            viz_msg.header = msg.header
            viz_msg.height = h
            viz_msg.width = w
            viz_msg.encoding = "bgr8"
            viz_msg.is_bigendian = 0
            viz_msg.step = w * 3
            viz_msg.data = viz_image.tobytes()
            self.viz_pub.publish(viz_msg)

        cmd = Twist()
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if tag_data is not None and dt > 0:
            # Steering: Error from center of image 
            # If tag is on the left (center_x < w/2), err_x is positive -> turn left (positive angular.z)
            err_x = (w / 2) - tag_data['center_x'] 
            cmd.angular.z = self.angular_pid.compute(err_x, dt)
            
            # Distance: Try to maintain safe_distance
            err_dist = tag_data['distance'] - self.safe_distance
            
            # Small deadband to prevent micro-oscillations when stopped
            if abs(err_dist) < 0.05:
                cmd.linear.x = 0.0
                self.linear_pid.integral = 0.0
            else:
                cmd.linear.x = self.linear_pid.compute(err_dist, dt)
                
        else:
            # Reset integrals if tag is lost to prevent windup
            self.linear_pid.integral = 0.0
            self.angular_pid.integral = 0.0
            
        # Always publish (stops the robot if no tag is found since cmd defaults to 0)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
