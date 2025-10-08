#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
#from rostopic import ROSTopicHz

class SquareNode:
    def __init__(self):
        self.LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left_thruster_thrust_cmd'
        self.RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right_thruster_thrust_cmd'
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
        rospy.on_shutdown(self.shutdown) ### TODO: check
        rospy.loginfo("Aguardando IMU...")
        self.current_yaw  = None
        self.imu_received = False

        self.state = None
        self.state_start_time = None
        self.side_counter = 0

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        if not self.imu_received:
            self.imu_received = True
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
            
            elif self.state == 'WAIT_TO_STOP':
                self.state_start_time = rospy.Time.now()
                rospy.loginfo_once(f"#--- Aguardando {self.WAIT_DURATION} segundos para o barco parar... ---#")

    def run(self):
        #hz_info = self.imu_rate_monitor.get_hz(self.IMU_TOPIC)

        
        #if self.imu_received and current_rate < self.MIN_IMU_HZ and self.state != 'IMU_FAILURE':
            #rospy.loginfo(f"FREQUÊNCIA DA IMU BAIXA: {current_rate:.2f} Hz. Parando o barco!")
            #self.change_state('IMU_FAILURE')
        
        if self.current_yaw is None:
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
            self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))
            # TODO: call manual mode service
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            rospy.loginfo_throttle(1, f"Alvo: {math.degrees(self.target_yaw):.1f}, Atual: {math.degrees(self.current_yaw):.1f}, Erro: {error_degrees:.1f}°")
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('FORWARD')
            else:
                self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN)
                
        elif self.state == 'WAIT_TO_STOP':
            self.stopping_the_boat()
            
            if self.state_start_time is None:
                raise Exception("ERRO: Faltando o start time!")

            elif (rospy.Time.now() - self.state_start_time >= rospy.Duration(self.WAIT_DURATION)):
                self.change_state('TURN')
        
        elif self.state == 'DONE':
            rospy.loginfo("Percurso do quadrado finalizado!")
            self.stopping_the_boat()

        # imu failed in operation condition
        elif self.state == 'IMU_FAILURE':
            self.stopping_the_boat()


def main():
    rospy.init_node('square_node', anonymous=True)
    rospy.loginfo("Nó 'square_node' inicializado.")
    controller = SquareNode() ### set rate on main
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
        rospy.sleep(0.5)

if __name__ == '__main__':
    main()
