#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Publishers for emergency commands
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.emergency_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', qos)
        
        # Subscribers for system monitoring
        self.depth_sub = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self.depth_monitor, 10)
        
        self.battery_sub = self.create_subscription(
            Float64, '/mavros/battery', self.battery_monitor, 10)
        
        # Service clients for emergency actions
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Safety thresholds
        self.max_depth = -5.0  # Maximum safe depth
        self.min_depth = -0.3  # Minimum safe depth (avoid surfacing)
        self.low_battery_threshold = 20.0  # 20% battery
        self.critical_battery_threshold = 10.0  # 10% battery
        
        # System state
        self.current_depth = 0.0
        self.battery_level = 100.0
        self.emergency_mode = False
        self.safety_violation_count = 0
        
        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.2, self.safety_check)
        self.heartbeat_timer = self.create_timer(1.0, self.system_heartbeat)
        
        self.get_logger().info("Safety Monitor initialized - Monitoring system health")

    def depth_monitor(self, msg):
        self.current_depth = msg.data

    def battery_monitor(self, msg):
        # Assuming battery message has percentage field
        if hasattr(msg, 'percentage'):
            self.battery_level = msg.percentage
        elif hasattr(msg, 'data'):
            self.battery_level = msg.data

    def safety_check(self):
        """Main safety monitoring function"""
        safety_issues = []
        
        # Depth safety check
        if self.current_depth < self.max_depth:
            safety_issues.append(f"CRITICAL: Exceeded max depth ({self.current_depth:.1f}m)")
            self.emergency_ascend()
            
        if self.current_depth > self.min_depth:
            safety_issues.append(f"CRITICAL: Too close to surface ({self.current_depth:.1f}m)")
            self.emergency_descend()
        
        # Battery safety check
        if self.battery_level < self.critical_battery_threshold:
            safety_issues.append(f"CRITICAL: Battery critically low ({self.battery_level:.1f}%)")
            self.emergency_surface()
        elif self.battery_level < self.low_battery_threshold:
            safety_issues.append(f"WARNING: Battery low ({self.battery_level:.1f}%)")
            self.conserve_power()
        
        # Log safety issues
        for issue in safety_issues:
            self.get_logger().error(issue)
            self.safety_violation_count += 1
        
        # Enter emergency mode if multiple violations
        if self.safety_violation_count > 5 and not self.emergency_mode:
            self.activate_emergency_mode()

    def activate_emergency_mode(self):
        """Activate full emergency mode"""
        self.emergency_mode = True
        self.get_logger().error("!!! EMERGENCY MODE ACTIVATED !!!")
        
        # Stop all movement and surface
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.3  # Slow ascent
        twist.angular.z = 0.0
        self.emergency_pub.publish(twist)

    def emergency_ascend(self):
        """Emergency ascent procedure"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.4  # Moderate ascent
        twist.angular.z = 0.0
        self.emergency_pub.publish(twist)

    def emergency_descend(self):
        """Emergency descent procedure"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = -0.3  # Controlled descent
        twist.angular.z = 0.0
        self.emergency_pub.publish(twist)

    def emergency_surface(self):
        """Emergency surface procedure for low battery"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.5  # Fast ascent to surface
        twist.angular.z = 0.0
        self.emergency_pub.publish(twist)

    def conserve_power(self):
        """Reduce power consumption when battery is low"""
        twist = Twist()
        twist.linear.x = 0.1  # Very slow forward
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.z = 0.1  # Very slow rotation
        self.emergency_pub.publish(twist)

    def system_heartbeat(self):
        """Periodic system status report"""
        if not self.emergency_mode:
            self.get_logger().info(
                f"System OK - Depth: {self.current_depth:.1f}m, "
                f"Battery: {self.battery_level:.1f}%, "
                f"Violations: {self.safety_violation_count}"
            )
        else:
            self.get_logger().error(
                f"EMERGENCY MODE - Depth: {self.current_depth:.1f}m, "
                f"Battery: {self.battery_level:.1f}%"
            )

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
