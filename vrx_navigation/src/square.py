#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from vrx_navigation.control_module import PIDController
# from control_module import PIDController
#from rostopic import ROSTopicHz

PID_KP = 2.0
PID_KI = 0.0
PID_KD = 1.3
PID_KS = 50 # error buffer

class SquareNode:
    def __init__(self):
        self.LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left_thrust_cmd'
        self.RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right_thrust_cmd'
        self.IMU_TOPIC = '/wamv/sensors/imu/imu/data'

        self.THRUST_NEUTRAL = 0.0
        self.THRUST_FORWARD = 20.0
        self.SIDE_DURATION = 4.0
        self.WAIT_DURATION = 2.0
        self.HOLD_DURATION = 5.0
        self.MAX_TURN_THRUST = 5.0
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

        self.last_imu_time = None
        self.imu_timeout = rospy.Duration(1.0)

        self.state = 'WAITING_FOR_IMU'
        self.state_start_time = None
        self.side_counter = 0

        self.pid_gains = (PID_KP, PID_KI, PID_KD, PID_KS)
        self.pid_turn = PIDController(*self.pid_gains)
        rospy.loginfo(f"Controlador PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")

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

    def stopping_the_boat(self):
        self.set_thrusters(self.THRUST_NEUTRAL, self.THRUST_NEUTRAL)

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
            
            rospy.loginfo("Resetando o controlador PID de guinada para a nova curva.")
            self.pid_turn = PIDController(*self.pid_gains)

        elif self.state == 'WAIT_TO_STOP':
            self.state_start_time = rospy.Time.now()
            rospy.loginfo(f"#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#")

        elif self.state == 'HOLD_TURN':
            self.state_start_time = rospy.Time.now()
            rospy.loginfo(f"Alvo alcançado. Mirando por {self.HOLD_DURATION}s...")
            self.stopping_the_boat() # Para os motores
    
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
            self.set_thrusters(self.THRUST_FORWARD, self.THRUST_FORWARD)
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
            rospy.loginfo_throttle(0.5, f"Alvo: {math.degrees(self.target_yaw):.1f}, Atual: {math.degrees(self.current_yaw):.1f}, Erro: {error_degrees:.1f}°")
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('HOLD_TURN')
            else:
                current_time = rospy.Time.now().to_sec()
                control_signal = self.pid_turn.control(error_rad, current_time)
                control_signal_saturated = max(-self.MAX_TURN_THRUST, min(self.MAX_TURN_THRUST, control_signal))

                left_thrust = -control_signal_saturated
                right_thrust = control_signal_saturated
                
                rospy.loginfo_throttle(0.5, f"PID Output: {control_signal:.2f} -> Sat: {control_signal_saturated:.2f} -> L: {left_thrust:.2f} R: {right_thrust:.2f}")
                self.set_thrusters(left_thrust, right_thrust)

        elif self.state == 'HOLD_TURN':
            self.stopping_the_boat()

            if (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.HOLD_DURATION)):
                self.change_state('FORWARD')

        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.WAIT_DURATION)):
                self.change_state('TURN')
        elif self.state == 'DONE':
            rospy.loginfo_once("Percurso do quadrado finalizado!")
            self.stopping_the_boat()

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