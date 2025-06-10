#!/usr/bin/env python3

# https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/
# https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
# https://docs.ros.org/en/melodic/api/robot_localization/html/index.html#
# https://github.com/cra-ros-pkg/robot_localization

import rclpy
import utm
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left/thrust'
RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right/thrust'

GPS_TOPIC = '/wamv/sensors/gps/gps/fix'
IMU_TOPIC = '/wamv/sensors/imu/imu/data'

THRUST_MIN = -1000.0
THRUST_MAX = 1000.0

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Waypoints
        self.declare_parameter('waypoints', [
            -33.722499, 150.673399,
        ]) 

        waypoints_flat = self.get_parameter('waypoints').value
        self.waypoints = [[waypoints_flat[i], waypoints_flat[i+1]] for i in range(0, len(waypoints_flat), 2)]
        self.declare_parameter('tolerance', 1.0)
        self.declare_parameter('thrust_scaling_factor', 600.0)
        self.tolerance = self.get_parameter('tolerance').value
        self.thrust_scale = self.get_parameter('thrust_scaling_factor').value
        
        self.current_waypoint_index = 0
        self.current_utm_position = None
        self.current_yaw = None
        self.utm_zone = None
        self.angular_gain = 1.0

        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, GPS_TOPIC, self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_callback, 10)

        # Publishers
        self.left_thrust_pub = self.create_publisher(Float64, LEFT_THRUSTER_TOPIC, 10)
        self.right_thrust_pub = self.create_publisher(Float64, RIGHT_THRUSTER_TOPIC, 10)

        self.timer = self.create_timer(0.1, self.navigate)
        self.get_logger().info(f"Número de: {len(self.waypoints)} waypoints.")
        self.get_logger().info("Aguardando dados de GPS e IMU...")

    def gps_callback(self, msg: NavSatFix):
        if self.utm_zone is None: # da primeira leitura, define a zona utm
            _ , _, self.utm_zone_number, self.utm_zone_letter = utm.from_latlon(msg.latitude, msg.longitude)

        utm_coords = utm.from_latlon(msg.latitude, msg.longitude, self.utm_zone_number, self.utm_zone_letter)
        self.current_utm_position = (utm_coords[0], utm_coords[1])

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def navigate(self):     
        if self.current_waypoint_index >= len(self.waypoints):
            self.set_thrusters(0.0, 0.0)
            if self.timer.is_canceled() is False:
                self.get_logger().info("Missão completa!")
                self.timer.cancel()
            return

        if self.current_utm_position is None or self.current_yaw is None:
            return

        target_lat, target_lon = self.waypoints[self.current_waypoint_index]
        target_utm = utm.from_latlon(target_lat, target_lon, self.utm_zone_number, self.utm_zone_letter)
        target_x, target_y = target_utm[0], target_utm[1]
        
        current_x, current_y = self.current_utm_position
        
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        
        if distance_to_target < self.tolerance:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} ({target_lat:.4f}, {target_lon:.4f}) alcançado!")
            self.current_waypoint_index += 1
            return

        angle_error = self.normalize_angle(angle_to_target - self.current_yaw)
        
        angular_vel_target = self.angular_gain * angle_error
        linear_vel_target = 1.0 if abs(angle_error) < 0.5 else 0.0

        forward_thrust = linear_vel_target * self.thrust_scale
        turn_thrust = angular_vel_target * self.thrust_scale
        
        left_target_thrust = forward_thrust - turn_thrust
        right_target_thrust = forward_thrust + turn_thrust
        
        self.set_thrusters(left_target_thrust, right_target_thrust)

    def set_thrusters(self, left_target, right_target):
        left_thrust = max(THRUST_MIN, min(THRUST_MAX, left_target))
        right_thrust = max(THRUST_MIN, min(THRUST_MAX, right_target))

        self.left_thrust_pub.publish(Float64(data=left_thrust))
        self.right_thrust_pub.publish(Float64(data=right_thrust))

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.get_logger().info('Parando motores antes de sair...')
        navigator.set_thrusters(0.0, 0.0)
        time.sleep(0.5)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()