#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64, Header
from geometry_msgs.msg import Twist, PoseStamped
from vision_msgs.msg import BoundingBoxArray
from mavros_msgs.srv import CommandBool, SetMode

class AutonomousUnderwaterVehicleController(Node):

    def __init__(self):
        super().__init__('autonomous_underwater_vehicle_controller')
        
        # System configuration parameters
        self.OPERATIONAL_DEPTH_TARGET = -2.0
        self.HORIZONTAL_ALIGNMENT_RANGE = (0.45, 0.55)
        self.VERTICAL_ALIGNMENT_RANGE = (0.45, 0.55)
        self.SETPOINT_STREAMING_COUNT_TARGET = 50
        self.GATE_MEMORY_DURATION = 3.0
        
        # System state tracking
        self.control_phase = 0
        self.vehicle_armed_status = False
        self.guided_operation_active = False
        self.depth_target_achieved = False
        self.vision_target_acquired = False
        self.initialization_sequence_complete = False
        self.setpoint_counter = 0
        
        # Navigation data
        self.active_target = None
        self.previous_target_data = None
        self.last_target_detection = None
        self.current_vehicle_depth = 0.0
        
        # Initialize communication interfaces
        self.setup_communication_interfaces()
        
        # Configure operational timers
        self.configure_operational_timers()
        
        self.get_logger().info("Autonomous Underwater Vehicle Controller initialized successfully")

    def setup_communication_interfaces(self):
        """Configure all publishers, subscribers, and service clients"""
        # Output interfaces
        self.motion_command_interface = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 20)
        self.position_target_interface = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 20)
        
        # Input interfaces
        self.vision_sensor_input = self.create_subscription(
            BoundingBoxArray, '/main_camera/detection/bounding_boxes',
            self.process_vision_sensor_data, 20)
        self.depth_sensor_input = self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self.process_depth_sensor_data, 20)
        
        # External service interfaces
        self.vehicle_arming_interface = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.operational_mode_interface = self.create_client(
            SetMode, '/mavros/set_mode')

    def configure_operational_timers(self):
        """Setup all operational timers for system control"""
        self.vehicle_initialization_timer = self.create_timer(0.5, self.execute_vehicle_initialization)
        self.velocity_management_timer = self.create_timer(0.1, self.manage_velocity_commands)
        self.target_processing_timer = self.create_timer(0.1, self.process_detected_targets)
        self.vertical_movement_timer = self.create_timer(0.1, self.execute_vertical_movement)
        self.horizontal_movement_timer = self.create_timer(0.1, self.execute_horizontal_movement)
        self.setpoint_streaming_timer = self.create_timer(0.1, self.execute_setpoint_streaming)
        self.navigation_management_timer = self.create_timer(0.1, self.manage_navigation_operations)

    def execute_vehicle_initialization(self):
        """Initialize vehicle systems including arming and mode setting"""
        if not self.vehicle_armed_status:
            if self.vehicle_arming_interface.service_is_ready():
                arming_request = CommandBool.Request()
                arming_request.value = True
                service_future = self.vehicle_arming_interface.call_async(arming_request)
                service_future.add_done_callback(self.handle_arming_service_response)
                self.get_logger().info('Initiating vehicle arming procedure...')
            return

        if not self.guided_operation_active:
            if self.operational_mode_interface.service_is_ready():
                mode_request = SetMode.Request()
                mode_request.custom_mode = 'GUIDED'
                service_future = self.operational_mode_interface.call_async(mode_request)
                service_future.add_done_callback(self.handle_mode_service_response)
                self.get_logger().info('Configuring GUIDED operational mode...')
            return

        self.vehicle_initialization_timer.cancel()
        self.get_logger().info('Vehicle initialization complete - Ready for mission execution')

    def handle_arming_service_response(self, service_future):
        """Process response from vehicle arming service"""
        try:
            service_response = service_future.result()
            if service_response.success:
                self.vehicle_armed_status = True
                self.get_logger().info('Vehicle arming confirmed - Systems operational')
            else:
                self.get_logger().warning('Vehicle arming procedure unsuccessful')
        except Exception as service_error:
            self.get_logger().error(f'Arming service communication failure: {service_error}')

    def handle_mode_service_response(self, service_future):
        """Process response from operational mode service"""
        try:
            service_response = service_future.result()
            if service_response.mode_sent:
                self.guided_operation_active = True
                self.get_logger().info('GUIDED operational mode activation confirmed')
            else:
                self.get_logger().warning('Operational mode configuration unsuccessful')
        except Exception as service_error:
            self.get_logger().error(f'Mode service communication failure: {service_error}')

    def process_vision_sensor_data(self, sensor_data):
        """Process incoming vision sensor data for target detection"""
        self.vision_target_acquired = False
        self.active_target = None
        
        if hasattr(sensor_data, 'bounding_boxes') and sensor_data.bounding_boxes:
            for detected_object in sensor_data.bounding_boxes:
                if (hasattr(detected_object, 'label_name') and 
                    'gate' in detected_object.label_name.lower()):
                    self.active_target = detected_object
                    self.vision_target_acquired = True
                    self.previous_target_data = detected_object
                    self.last_target_detection = self.get_clock().now()
                    self.get_logger().info(
                        f'Target identified - Coordinates: '
                        f'X={detected_object.x:.3f}, Y={detected_object.y:.3f}, '
                        f'Dimensions: {detected_object.w:.3f}x{detected_object.h:.3f}'
                    )
                    break

    def process_detected_targets(self):
        """Process detected targets for horizontal alignment"""
        if (self.control_phase == 1 and self.vision_target_acquired == True):
            target_horizontal_position = self.active_target.x
            alignment_min, alignment_max = self.HORIZONTAL_ALIGNMENT_RANGE
            
            if not (alignment_min < target_horizontal_position < alignment_max):
                movement_command = Twist()
                movement_command.linear.x = 0.0
                movement_command.linear.y = 0.0
                movement_command.linear.z = 0.0
                
                if target_horizontal_position > alignment_max:
                    self.get_logger().info(f"Target horizontal position: {target_horizontal_position:.3f}")
                    movement_command.angular.z = -0.1
                    self.motion_command_interface.publish(movement_command)
                    self.get_logger().info("Executing rightward horizontal alignment")
                else:
                    self.get_logger().info(f"Target horizontal position: {target_horizontal_position:.3f}")
                    movement_command.angular.z = 0.1
                    self.motion_command_interface.publish(movement_command)
                    self.get_logger().info("Executing leftward horizontal alignment")
                return
            
            if (alignment_min < target_horizontal_position < alignment_max):
                self.get_logger().info("Horizontal alignment objective completed")
                self.control_phase = 2
                self.velocity_management_timer.cancel()
                return

    def execute_vertical_movement(self):
        """Execute vertical movement for target alignment"""
        if (self.control_phase == 2 and self.vision_target_acquired == True):
            target_vertical_position = self.active_target.y
            alignment_min, alignment_max = self.VERTICAL_ALIGNMENT_RANGE
            
            if not (alignment_min < target_vertical_position < alignment_max):
                self.get_logger().info(f"Target vertical position: {target_vertical_position:.3f}")
                vertical_movement_command = Twist()
                vertical_movement_command.linear.x = 0.0
                vertical_movement_command.linear.y = 0.0
                vertical_movement_command.linear.z = -0.5
                vertical_movement_command.angular.z = 0.0
                self.motion_command_interface.publish(vertical_movement_command)
                self.get_logger().info("Executing downward vertical alignment")
                return
            
            if (alignment_min < target_vertical_position < alignment_max):
                self.get_logger().info("Vertical alignment objective completed")
                self.control_phase = 3
                self.vertical_movement_timer.cancel()
                return

    def execute_horizontal_movement(self):
        """Execute forward movement through target"""
        if (self.control_phase == 3 and self.vision_target_acquired == True):
            if not self.vision_target_acquired:
                self.get_logger().info("Target traversal sequence completed")
                self.horizontal_movement_timer.cancel()
                return
            
            forward_movement_command = Twist()
            forward_movement_command.linear.x = 0.5
            forward_movement_command.linear.y = 0.0
            forward_movement_command.linear.z = 0.0
            forward_movement_command.angular.z = 0.0
            self.motion_command_interface.publish(forward_movement_command)
            self.get_logger().info("Executing forward movement through target")
            return

    def manage_velocity_commands(self):
        """Manage velocity commands based on system state"""
        if not (self.vehicle_armed_status and self.guided_operation_active):
            return
        
        if self.vision_target_acquired and self.active_target:
            stabilization_command = Twist()
            stabilization_command.linear.x = 0.0
            stabilization_command.linear.y = 0.0
            stabilization_command.linear.z = 0.0
            stabilization_command.angular.z = 0.0
            self.motion_command_interface.publish(stabilization_command)
            self.get_logger().info("Target acquisition confirmed - Initiating alignment sequence")
            self.control_phase = 1
            self.velocity_management_timer.cancel()
            return
        
        if not (self.vision_target_acquired and self.active_target and self.control_phase == 1):
            search_pattern_command = Twist()
            search_pattern_command.linear.x = 0.0
            search_pattern_command.linear.y = 0.0
            search_pattern_command.linear.z = 0.0
            search_pattern_command.angular.z = 0.5
            self.motion_command_interface.publish(search_pattern_command)
            self.get_logger().info("Executing target search pattern")

    def execute_setpoint_streaming(self):
        """Execute setpoint streaming for control system initialization"""
        if self.setpoint_counter < self.SETPOINT_STREAMING_COUNT_TARGET:
            neutral_velocity_command = Twist()
            self.motion_command_interface.publish(neutral_velocity_command)
            self.setpoint_counter += 1
            if self.setpoint_counter == 1:
                self.get_logger().info('Initializing setpoint streaming sequence...')
        elif not self.initialization_sequence_complete:
            self.configure_guided_operational_mode()
            self.initiate_vehicle_arming()
            self.initialization_sequence_complete = True
            self.get_logger().info('Setpoint streaming sequence completed successfully')
            self.setpoint_streaming_timer.cancel()

    def configure_guided_operational_mode(self):
        """Configure GUIDED operational mode"""
        if not self.operational_mode_interface.service_is_ready():
            self.get_logger().info('Waiting for operational mode service availability...')
            return
        mode_configuration_request = SetMode.Request()
        mode_configuration_request.custom_mode = 'GUIDED'
        self.operational_mode_interface.call_async(mode_configuration_request)

    def initiate_vehicle_arming(self):
        """Initiate vehicle arming sequence"""
        if not self.vehicle_arming_interface.service_is_ready():
            self.get_logger().info('Waiting for vehicle arming service availability...')
            return
        arming_sequence_request = CommandBool.Request()
        arming_sequence_request.value = True
        self.vehicle_arming_interface.call_async(arming_sequence_request)
        self.vehicle_armed_status = True
        self.get_logger().info('Vehicle arming sequence initiated')

    def process_depth_sensor_data(self, sensor_data):
        """Process depth sensor data for depth control"""
        self.current_vehicle_depth = sensor_data.data
        if not self.depth_target_achieved and self.current_vehicle_depth <= self.OPERATIONAL_DEPTH_TARGET + 0.1:
            self.depth_target_achieved = True
            self.get_logger().info('Operational depth target achieved successfully!')

    def navigate_to_operational_depth(self):
        """Navigate to operational depth target"""
        depth_target_configuration = PoseStamped()
        depth_target_configuration.pose.position.x = 0.0
        depth_target_configuration.pose.position.y = 0.0
        depth_target_configuration.pose.position.z = self.OPERATIONAL_DEPTH_TARGET
        self.position_target_interface.publish(depth_target_configuration)
        self.get_logger().info(
            f'Depth navigation active - Target: {self.OPERATIONAL_DEPTH_TARGET}m, '
            f'Current: {self.current_vehicle_depth:.2f}m')

    def manage_navigation_operations(self):
        """Manage overall navigation operations"""
        # Phase 1: Depth navigation
        if not self.depth_target_achieved:
            self.navigate_to_operational_depth()
            return

        navigation_command = Twist()
        
        # Phase 2: Target navigation with persistence
        navigation_target = None
        if self.active_target:
            navigation_target = self.active_target
        elif self.previous_target_data and self.last_target_detection:
            time_since_last_detection = (self.get_clock().now() - self.last_target_detection).nanoseconds / 1e9
            if time_since_last_detection < self.GATE_MEMORY_DURATION:
                navigation_target = self.previous_target_data

        if navigation_target:
            target_center_calculation = navigation_target.x + navigation_target.w / 2.0
            horizontal_error_calculation = 0.5 - target_center_calculation
            rotational_gain_factor = 1.5
            navigation_command.angular.z = rotational_gain_factor * horizontal_error_calculation
            navigation_command.linear.x = 0.5
            self.get_logger().info(f'Target navigation active - Horizontal error: {horizontal_error_calculation:.2f}')
        else:
            # Phase 3: Search operations
            navigation_command.angular.z = 0.3
            self.get_logger().info('Executing target search operations...')

        self.motion_command_interface.publish(navigation_command)

def main(args=None):
    rclpy.init(args=args)
    auv_controller_instance = AutonomousUnderwaterVehicleController()
    try:
        rclpy.spin(auv_controller_instance)
    except KeyboardInterrupt:
        auv_controller_instance.get_logger().info('AUV Controller shutdown initiated')
    finally:
        auv_controller_instance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
