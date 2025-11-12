#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from vrx_navigation.cfg import NavigationConfig
from vrx_navigation.control_module import PIDController
#from rostopic import ROSTopicHz

class SquareNode:
    def __init__(self):
        self.LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left_thrust_cmd'
        self.RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right_thrust_cmd'
        self.IMU_TOPIC = '/wamv/sensors/imu/imu/data'

        self.THRUST_NEUTRAL = 0.0
        self.THRUST_FORWARD = 20.0
        self.SIDE_DURATION = 4.0
        self.WAIT_DURATION = 2.0
        
        self.THRUST_TURN = 0.5
        self.DEGREE_RANGE = 5.0

        self.left_thrust_pub = rospy.Publisher(self.LEFT_THRUSTER_TOPIC, Float32, queue_size=1)
        self.right_thrust_pub = rospy.Publisher(self.RIGHT_THRUSTER_TOPIC, Float32, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_callback)
        '''
        self.MIN_IMU_HZ = 10.0
        self.imu_rate_monitor = ROSTopicHz(15, filter_expr=None)
        self.imu_rate_sub = rospy.Subscriber(self.IMU_TOPIC, rospy.AnyMsg, self.imu_rate_monitor.callback_hz)
        '''
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Aguardando IMU...")
        self.current_yaw  = None
        self.target_yaw = None
        # self.imu_received = False

        self.last_imu_time = None
        self.imu_timeout = rospy.Duration(1.0)

        self.state = 'WAITING_FOR_IMU'
        self.state_start_time = None
        self.side_counter = 0

    def imu_callback(self, msg: Imu):
        self.last_imu_time = rospy.Time.now()
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        if self.state == 'WAITING_FOR_IMU':
            rospy.loginfo("Dados da IMU recebidos!")
            self.change_state('FORWARD')

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def set_thrusters(self, left_thrust, right_thrust):
        self.left_thrust_pub.publish(Float32(float(left_thrust)))
        self.right_thrust_pub.publish(Float32(float(right_thrust)))

    def orientation_control(self, angle):
        # from SINGABOAT-VRX
        # change to angle logic
        # server?
        err_pos = self.cmd_pos - self.cur_pos # Error in position [x_des - x_cur, y_des - y_cur]
        lin_vel_x = self.pid_sk_vx.control(err_pos[0], self.time) # PID controller for Vx
        lin_vel_y = self.pid_sk_vy.control(err_pos[1], self.time) # PID controller for Vy
        # Generate and publish `cmd_vel` message
        self.cmd_vel_msg.linear.x = lin_vel_x
        self.cmd_vel_msg.linear.y = lin_vel_y
        #self.cmd_vel_msg.angular.z = ang_vel_z
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.thr_1 = 0 # u > to thruster 1
        self.thr_2 = 0 # u > to thruster 2
        ### or just call set_thrusters
        return self.thr_1, self.thr_2

    def stopping_the_boat(self):
        self.set_thrusters(self.THRUST_NEUTRAL, self.THRUST_NEUTRAL)
        # TODO: call loiter mode service

    def shutdown(self):
        rospy.loginfo("Barco parado, desligando o nó.")
        self.stopping_the_boat()

    def change_state(self, new_state):
        if self.state != new_state:
            rospy.loginfo(f"Mudando de estado: {self.state} -> {new_state}")
            self.state = new_state

            if self.state == 'FORWARD':
                self.state_start_time = rospy.Time.now()
                rospy.loginfo(f"#--- LADO {self.side_counter + 1}: Iniciando movimento em linha reta... ---#")
            
            elif self.state == 'TURN':
                self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))

            elif self.state == 'WAIT_TO_STOP':
                self.state_start_time = rospy.Time.now()
                rospy.loginfo_once(f"#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#")

    def config_callback(self, config, level): # level?
        # from SINGABOAT-VRX
        # Handle updated configuration values
        self.gps_offset = config['gps_offset'] # GPS offset w.r.t. WAM-V along X-axis
        self.pid_g2g    = PIDController(config['G2G_kP'], config['G2G_kI'], config['G2G_kD'], config['G2G_kS']) # Go-to-goal PID controller
        self.pid_sk_vx  = PIDController(config['SK_Vx_kP'], config['SK_Vx_kI'], config['SK_Vx_kD'], config['SK_Vx_kS']) # Station-keeping Vx PID controller
        self.pid_sk_vy  = PIDController(config['SK_Vy_kP'], config['SK_Vy_kI'], config['SK_Vy_kD'], config['SK_Vy_kS']) # Station-keeping Vy PID controller
        self.pid_sk_wz  = PIDController(config['SK_Wz_kP'], config['SK_Wz_kI'], config['SK_Wz_kD'], config['SK_Wz_kS']) # Station-keeping Wz PID controller
        self.config     = config
        return config

    def run(self):
        #hz_info = self.imu_rate_monitor.get_hz(self.IMU_TOPIC)

        
        #if self.imu_received and current_rate < self.MIN_IMU_HZ and self.state != 'IMU_FAILURE':
            #rospy.loginfo(f"FREQUÊNCIA DA IMU BAIXA: {current_rate:.2f} Hz. Parando o barco!")
            #self.change_state('IMU_FAILURE')
        
        if self.state == 'WAITING_FOR_IMU':
            rospy.loginfo_once("Aguardando a primeira mensagem da IMU...")
            return

        if self.last_imu_time is None or (rospy.Time.now() - self.last_imu_time) > self.imu_timeout:
            self.change_state('IMU_FAILURE')
            raise Exception("ERRO: Dados da IMU não estão sendo recebidos!")
        

        if self.state == 'FORWARD':
            # call control loop
            self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN) # set into control loop
            # TODO: call manual mode service
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.SIDE_DURATION)):
                self.side_counter += 1
                if self.side_counter >= 4:
                    self.change_state('DONE')
                else:
                    self.change_state('WAIT_TO_STOP')

        elif self.state == 'TURN':
            # TODO: call manual mode service
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            rospy.loginfo_throttle(1, f"Alvo: {math.degrees(self.target_yaw):.1f}, Atual: {math.degrees(self.current_yaw):.1f}, Erro: {error_degrees:.1f}°")
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('FORWARD')
            else:
                # call control loop
                self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN) # set into control loop
                
        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.WAIT_DURATION)):
                self.change_state('TURN')
        
        elif self.state == 'DONE':
            rospy.loginfo_once("Percurso do quadrado finalizado!")
            self.stopping_the_boat()

        # imu failed in operation condition
        elif self.state == 'IMU_FAILURE':
            self.stopping_the_boat()


def main():
    rospy.init_node('square_node', anonymous=True)
    rospy.loginfo("Nó 'square_node' inicializado.")
    controller = SquareNode()
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            controller.run()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido (Ctrl+C).")
        pass

    finally:
        rospy.loginfo('Parando o barco...')
        controller.stopping_the_boat()
        
if __name__ == '__main__':
    main()

'''
from https://github.com/hydrone-furg/vrx/blob/jazzy/vrx_navigation/vrx_navigation/test/loiter_node.py

    def control_loop(self):
        if self.current_latitude is None or self.current_longitude is None or self.current_yaw_rad is None:
            self.set_thruster_commands(0.0, 0.0) # Thrusters sao desligados se nao houver dados

            return

        if not self.loiter_center_set:
            self.set_thruster_commands(0.0, 0.0) # Mantem os thrusters desligados ate o instante que o centro seja definido

            return

        distance_to_target, bearing_to_target_rad = self.calculate_distance_and_bearing(
            self.current_latitude, self.current_longitude,
            self.target_lat, self.target_lon
        )

        current_yaw_ned_rad = self.normalize_angle_mpi_pi((math.pi / 2.0) - self.current_yaw_rad) # Em NED
        error_yaw_rad = self.normalize_angle_mpi_pi(bearing_to_target_rad - current_yaw_ned_rad)

        # Log #####
        self.get_logger().info(
             f"Loiter: D={distance_to_target:.1f}m, AzTgt(NED)={math.degrees(bearing_to_target_rad):.1f}°, "
             f"YawCurr(NED)={math.degrees(current_yaw_ned_rad):.1f}°, ErrYaw={math.degrees(error_yaw_rad):.1f}°",
             throttle_duration_sec=0.5
        ) #####

        # P Controller
        turn_control_effort = KP_YAW_LOITER * error_yaw_rad
        turn_control_effort = max(-MAX_THRUST_DIFFERENTIAL_LOITER, min(MAX_THRUST_DIFFERENTIAL_LOITER, turn_control_effort))
        forward_thrust_command = 0.0
        if distance_to_target > LOITER_DEADBAND:
            forward_thrust_command = KP_DISTANCE_LOITER * distance_to_target
            forward_thrust_command = min(forward_thrust_command, MAX_FORWARD_THRUST_LOITER)
            if abs(error_yaw_rad) > ALIGNED_YAW_ERROR_THRESHOLD_RAD_LOITER: # Correcao?
                alignment_factor = max(0.0, (math.pi - abs(error_yaw_rad)) / math.pi)
                forward_thrust_command *= alignment_factor**2 

        left_command = forward_thrust_command + turn_control_effort
        right_command = forward_thrust_command - turn_control_effort

        self.set_thruster_commands(left_command, right_command)

        if distance_to_target <= LOITER_DEADBAND and self.loiter_center_set:
             self.get_logger().info(f"Mantendo posiçao. Dist: {distance_to_target:.2f}m", throttle_duration_sec=5.0)

'''