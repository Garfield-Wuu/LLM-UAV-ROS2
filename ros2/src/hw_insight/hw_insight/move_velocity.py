#!/usr/bin/env python3
"""
MoveVelocity — low-level PX4 Offboard velocity controller.

Safety gate: NO offboard signals (OffboardControlMode / TrajectorySetpoint /
VehicleCommand) are sent to PX4 until flight has been explicitly requested
(i.e. until hw_velocity.z < ARM_TRIGGER_THRESHOLD is received).  This
prevents the drone from arming or switching modes the moment the ROS stack
is launched.

Once flight is requested the node maintains the Offboard heartbeat for the
full flight lifetime.  The gate resets automatically after the drone
disarms, so the next TAKEOFF starts clean.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Range


ARM_TRIGGER_THRESHOLD = -5.0


class MoveVelocity(Node):
    def __init__(self) -> None:
        super().__init__('move_velocity')
        self.declare_parameter('command_topic', '/hw_insight/keyboard_velocity')
        command_topic = str(self.get_parameter('command_topic').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )

        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.distance_sensor_subscriber = self.create_subscription(
            Range, '/airsim_node/PX4/distance/DistanceDown', self.distance_sensor_callback, 1
        )
        self.hw_position_subscriber = self.create_subscription(
            HWSimpleKeyboardInfo,
            command_topic,
            self.hw_velocity_callback,
            1,
        )

        self.vehicle_status = VehicleStatus()
        self.distance_sensor = Range()
        self.hw_velocity = HWSimpleKeyboardInfo()

        # Safety gate: only send PX4 signals when the operator has explicitly
        # requested flight (TAKEOFF command delivers z < ARM_TRIGGER_THRESHOLD).
        self.flight_enabled = False
        self._prev_arming_state: int = 0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'MoveVelocity command topic: {command_topic}')

    def vehicle_status_callback(self, vehicle_status: VehicleStatus) -> None:
        prev = self._prev_arming_state
        self.vehicle_status = vehicle_status
        cur = int(vehicle_status.arming_state)
        # When the drone disarms (transitions back to DISARMED) and we are
        # not being actively commanded to fly, release the gate so the next
        # TAKEOFF starts from a clean state.
        if (
            cur == VehicleStatus.ARMING_STATE_DISARMED
            and prev != VehicleStatus.ARMING_STATE_DISARMED
            and self.hw_velocity.z >= ARM_TRIGGER_THRESHOLD
        ):
            self.flight_enabled = False
            self.get_logger().info('Drone disarmed — flight_enabled reset to False')
        self._prev_arming_state = cur

    def distance_sensor_callback(self, distance_sensor: Range) -> None:
        self.distance_sensor = distance_sensor

    def hw_velocity_callback(self, hw_velocity: HWSimpleKeyboardInfo) -> None:
        self.hw_velocity = hw_velocity

    def arm(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_home(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

    def publish_offboard_control_heartbeat_signal(self) -> None:
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self, x: float, y: float, z: float, yawspeed: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [x, y, z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        # AirSim+PX4 SITL uses opposite yawspeed sign from standard NED convention:
        # positive yawspeed in this setup = counter-clockwise (left), so negate.
        msg.yawspeed = -yawspeed
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        # ── Safety gate ──────────────────────────────────────────────────────
        # Do NOT send any signal to PX4 until the operator explicitly commands
        # a takeoff (z < ARM_TRIGGER_THRESHOLD).  This prevents the drone from
        # arming or mode-switching the moment the ROS stack is launched.
        if not self.flight_enabled:
            if self.hw_velocity.z < ARM_TRIGGER_THRESHOLD:
                self.flight_enabled = True
                self.get_logger().info('Flight requested — activating Offboard stream')
            else:
                return  # Strict silence: nothing goes to PX4
        # ─────────────────────────────────────────────────────────────────────

        self.publish_offboard_control_heartbeat_signal()

        arming = self.vehicle_status.arming_state
        nav = self.vehicle_status.nav_state

        if arming == VehicleStatus.ARMING_STATE_DISARMED:
            # Drone is disarmed. A z < threshold triggers arm + Offboard switch.
            if self.hw_velocity.z < ARM_TRIGGER_THRESHOLD:
                self.engage_offboard_mode()
                self.arm()
                self.publish_velocity_setpoint(
                    self.hw_velocity.x,
                    self.hw_velocity.y,
                    self.hw_velocity.z,
                    self.hw_velocity.yaw,
                )

        elif nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Normal Offboard flight: relay velocity commands.
            if self.hw_velocity.z > 5 and self.distance_sensor.range < 5:
                self.land()
            else:
                self.publish_velocity_setpoint(
                    self.hw_velocity.x,
                    self.hw_velocity.y,
                    self.hw_velocity.z,
                    self.hw_velocity.yaw,
                )

        elif nav == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # Landing in progress.  A strong upward command aborts landing.
            if self.hw_velocity.z < ARM_TRIGGER_THRESHOLD:
                self.engage_offboard_mode()
                self.publish_velocity_setpoint(
                    self.hw_velocity.x,
                    self.hw_velocity.y,
                    self.hw_velocity.z,
                    self.hw_velocity.yaw,
                )

        else:
            # Mode-switch transition (e.g. between Offboard mode switch and
            # confirmation).  Keep the heartbeat alive and send the commanded
            # velocity so PX4 has valid setpoints when it accepts Offboard.
            if self.hw_velocity.z < ARM_TRIGGER_THRESHOLD:
                self.engage_offboard_mode()
                self.publish_velocity_setpoint(
                    self.hw_velocity.x,
                    self.hw_velocity.y,
                    self.hw_velocity.z,
                    self.hw_velocity.yaw,
                )
            else:
                self.publish_velocity_setpoint(0.0, 0.0, 0.0, 0.0)


def main(args=None) -> None:
    print('Starting move velocity node...')
    rclpy.init(args=args)
    move_velocity = MoveVelocity()
    rclpy.spin(move_velocity)
    move_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
