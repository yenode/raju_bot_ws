import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Robot physical parameters from URDF
        self.wheel_radius = 0.035
        self.track_width = 0.233  # 0.1165 * 2
        
        # Pose and state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_pos_prev = None
        self.right_pos_prev = None
        
        # ROS 2 Interfaces
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("Custom Odometry node initialized.")

    def joint_state_callback(self, msg):
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return  # The required joints are not in this message

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]
        left_vel = msg.velocity[left_idx]
        right_vel = msg.velocity[right_idx]
        
        if self.left_pos_prev is None or self.right_pos_prev is None:
            self.left_pos_prev = left_pos
            self.right_pos_prev = right_pos
            self.last_time = self.get_clock().now()
            return
            
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Calculate distances moved by each wheel
        d_left = (left_pos - self.left_pos_prev) * self.wheel_radius
        d_right = (right_pos - self.right_pos_prev) * self.wheel_radius
        
        self.left_pos_prev = left_pos
        self.right_pos_prev = right_pos
        self.last_time = current_time

        # Calculate robot kinematics
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.track_width

        # Update pose (using Runge-Kutta 2nd order for better accuracy)
        self.x += d_center * math.cos(self.theta + (d_theta / 2.0))
        self.y += d_center * math.sin(self.theta + (d_theta / 2.0))
        self.theta += d_theta

        # Calculate velocities
        v_linear = (left_vel * self.wheel_radius + right_vel * self.wheel_radius) / 2.0
        v_angular = (right_vel * self.wheel_radius - left_vel * self.wheel_radius) / self.track_width

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = euler_to_quaternion(0, 0, self.theta)
        
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.angular.z = v_angular
        
        self.odom_pub.publish(odom_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
