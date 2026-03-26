#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, DistanceSensor
from hw_interface.msg import HWSimpleKeyboardInfo


class MovePosition(Node):
    def __init__(self) -> None:
        super().__init__('move_position')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_local_postion_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_callback, qos_profile)
        self.distance_sensor_subscriber = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor', self.distance_sensor_callback, qos_profile)
        self.hw_position_subscriber = self.create_subscription(
            HWSimpleKeyboardInfo, '/hw_insight/keyboard_postion', self.hw_position_callback, 1)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.distance_sensor = DistanceSensor()
        self.hw_postion = HWSimpleKeyboardInfo()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
        #print(f"pos={self.vehicle_local_position.x},{self.vehicle_local_position.y},{self.vehicle_local_position.z}")

    def distance_sensor_callback(self, distance_sensor):
        self.distance_sensor = distance_sensor
        #print(f"current_distance={self.distance_sensor.current_distance}")

    def hw_position_callback(self, hw_position):
        self.hw_postion = hw_position
        #print(f"x={self.hw_postion.x}, y={self.hw_postion.y}, z={self.hw_postion.z}")

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def return_home(self):
        """Switch to return mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Switching to return mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        #print(f"nav_state = {self.vehicle_status.nav_state}, arm = {self.vehicle_status.arming_state}")

        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            # 未解锁状态，收到上升命令，则解锁并起飞，切换到OFFBOARD状态
            if self.hw_postion.z < (self.vehicle_local_position.z-2):
                self.engage_offboard_mode()
                self.arm()
                self.publish_position_setpoint(self.hw_postion.x, self.hw_postion.y, self.hw_postion.z, self.hw_postion.yaw)
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 已经解锁，且在OFFBOARD状态，则根据订阅/hw_simple_ned_postion得到的值控制无人机
            if (self.hw_postion.z > 2) or (self.distance_sensor.current_distance<5 and self.hw_postion.z > (self.vehicle_local_position.z+2)):
                # 降低到一定高度，出发降落指令
                self.land()
            else:
                # 按照命令飞行
                self.publish_position_setpoint(self.hw_postion.x, self.hw_postion.y, self.hw_postion.z, self.hw_postion.yaw)           
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # 降落状态下，如果收到上升指令，则结束降落状态，切换到OFFBOARD，并重新飞行
            if self.hw_postion.z < (self.vehicle_local_position.z-3):
                self.engage_offboard_mode()
                self.publish_position_setpoint(self.hw_postion.x, self.hw_postion.y, self.hw_postion.z, self.hw_postion.yaw)
        else:
            # 其他状态，正常逻辑不应该会到该状态，报错并返回
            self.land()
            #self.return_home()


        '''
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.hw_postion.z > 1:
                self.land()
            else:
                self.publish_position_setpoint(self.hw_postion.x, self.hw_postion.y, self.hw_postion.z)
        '''



def main(args=None) -> None:
    print('Starting keyboard move node...')
    rclpy.init(args=args)
    move_position = MovePosition()
    rclpy.spin(move_position)
    move_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
