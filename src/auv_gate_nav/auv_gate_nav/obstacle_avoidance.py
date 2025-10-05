#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Publisher for emergency commands
        self.cmd_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Subscriber for obstacle detection
        self.vision_sub = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.obstacle_callback,
            10
        )
        
        # Subscriber for depth (to avoid surface/floor)
        self.depth_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.depth_callback,
            10
        )
        
        # Obstacle detection state
        self.obstacle_detected = False
        self.obstacle_type = None
        self.obstacle_size = 0.0
        self.current_depth = 0.0
        self.safe_depth_min = -3.0  # Don't go too deep
        self.safe_depth_max = -0.5  # Don't surface
        
        # Timer for continuous monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_monitor)
        
        self.get_logger().info("Obstacle Avoider started - Monitoring for obstacles")

    def obstacle_callback(self, msg):
        self.obstacle_detected = False
        self.obstacle_type = None
        self.obstacle_size = 0.0
        
        if hasattr(msg, 'bounding_boxes') and msg.bounding_boxes:
            for box in msg.bounding_boxes:
                # Check for obstacles (not gates)
                if (hasattr(box, 'label_name') and 
                    any(obs in box.label_name.lower() for obs in ['obstacle', 'rock', 'wall', 'debris', 'danger'])):
                    
                    self.obstacle_detected = True
                    self.obstacle_type = box.label_name
                    self.obstacle_size = max(box.w, box.h)
                    
                    self.get_logger().warn(
                        f'OBSTACLE DETECTED: {box.label_name} '
                        f'(size: {self.obstacle_size:.2f}) at '
                        f'x={box.x:.2f}, y={box.y:.2f}'
                    )
                    break

    def depth_callback(self, msg):
        self.current_depth = msg.data

    def safety_monitor(self):
        # Check for depth safety
        if self.current_depth > self.safe_depth_max:
            self.get_logger().warn("TOO CLOSE TO SURFACE! Descending...")
            self.emergency_descend()
            return
            
        if self.current_depth < self.safe_depth_min:
            self.get_logger().warn("TOO DEEP! Ascending...")
            self.emergency_ascend()
            return
        
        # Check for obstacles
        if self.obstacle_detected:
            if self.obstacle_size > 0.3:  # Large obstacle - emergency maneuver
                self.get_logger().error("LARGE OBSTACLE! Executing emergency avoidance!")
                self.emergency_avoidance()
            elif self.obstacle_size > 0.15:  # Medium obstacle - careful avoidance
                self.get_logger().warn("Obstacle detected - avoiding carefully")
                self.careful_avoidance()
            else:  # Small obstacle - minor adjustment
                self.get_logger().info("Small obstacle - minor course correction")
                self.minor_correction()

    def emergency_avoidance(self):
        """Emergency maneuver for large obstacles"""
        twist = Twist()
        twist.linear.x = -0.2  # Move backward
        twist.linear.z = -0.3  # Descend
        twist.angular.z = 0.5  # Turn right
        self.cmd_pub.publish(twist)

    def careful_avoidance(self):
        """Careful avoidance for medium obstacles"""
        twist = Twist()
        twist.linear.x = 0.1   # Slow forward
        twist.linear.z = -0.2  # Descend slightly
        twist.angular.z = 0.3  # Turn right
        self.cmd_pub.publish(twist)

    def minor_correction(self):
        """Minor course correction for small obstacles"""
        twist = Twist()
        twist.linear.x = 0.2   # Reduced forward speed
        twist.linear.z = 0.0
        twist.angular.z = 0.2  # Slight turn
        self.cmd_pub.publish(twist)

    def emergency_descend(self):
        """Emergency descent when too close to surface"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.z = -0.5  # Fast descent
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def emergency_ascend(self):
        """Emergency ascent when too deep"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.z = 0.3   # Controlled ascent
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
