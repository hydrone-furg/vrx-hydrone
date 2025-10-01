#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

class SquareNode:
    def __init__(self):
        self.LEFT_THRUSTER_TOPIC = '/wamv/thrusters/left_thruster_thrust_cmd'
        self.RIGHT_THRUSTER_TOPIC = '/wamv/thrusters/right_thruster_thrust_cmd'
        self.IMU_TOPIC = '/wamv/sensors/imu/imu/data'

        self.THRUST_NEUTRAL = 0.0
        self.THRUST_FORWARD = 20.0
        self.SIDE_DURATION = 4.0
        
        self.THRUST_TURN = 0.5
        self.DEGREE_RANGE = 5.0

        rospy.init_node('square_node', anonymous=True)
        self.left_thrust_pub = rospy.Publisher(self.LEFT_THRUSTER_TOPIC, Float32, queue_size=1)
        self.right_thrust_pub = rospy.Publisher(self.RIGHT_THRUSTER_TOPIC, Float32, queue_size=1)
        self.imu_sub = rospy.Subscriber(self.IMU_TOPIC, Imu, self.imu_callback)

        rospy.on_shutdown(self.shutdown) ### TODO: check
        rospy.loginfo("Aguardando IMU...") ### TODO: check
        self.current_yaw  = None
        self.imu_received = False

        self.state = None
        self.state_start_time = None
        self.side_counter = 0

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        if not self.imu_received: # TODO: check
            self.imu_received = True
            rospy.loginfo("Dados da IMU recebidos!")
            self.change_state('FORWARD')
        else:
            self.change_state('WAITING_FOR_IMU') # TODO: check

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def set_thrusters(self, left_thrust, right_thrust):
        self.left_thrust_pub.publish(Float32(float(left_thrust)))
        self.right_thrust_pub.publish(Float32(float(right_thrust)))

    def stop_the_boat(self):
        self.set_thrusters(self.THRUST_NEUTRAL, self.THRUST_NEUTRAL)
        # TODO: call loiter mode service

    def shutdown(self):
        rospy.loginfo("Barco parado, desligando o nó.")
        self.stop_the_boat()

    '''
    def turn(self, target_angle):
        rate = rospy.Rate(20) ### set rate?
        rospy.loginfo(f"Iniciando virada até atingir o ponto de antecipação de {self.DEGREE_RANGE} graus.")
        ## set shutdown?
        error_rad = self.normalize_angle(target_angle - self.current_yaw )
        error_degrees = math.degrees(error_rad)

        rospy.loginfo_throttle(1, f"Alvo: {math.degrees(target_angle):.1f}, Erro: {error_degrees:.1f} graus")

        if abs(error_degrees) <= self.DEGREE_RANGE:
            rospy.loginfo("Condição satisfeita.")
            return

        self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN)
        rate.sleep()
    '''

    def state(self): # TODO: check ambiguity
        if self.state == 'WAITING_FOR_IMU':
            rospy.loginfo_once("Aguardando IMU...") 
            ### TODO: raise in Exception rospy.ROSInterruptException: (losing IMU data in operation)
            rospy.sleep(0.5)
            
        elif self.state == 'FORWARD': 
            self.run()
        
        elif self.state == 'TURN':
            self.turn(self.target_yaw)
            self.stop_the_boat()
            rospy.loginfo("Curva completa. Movendo para o próximo lado...")
            self.change_state('FORWARD')

    def change_state(self, new_state):
        rospy.loginfo(f"Mudando de estado: {self.state} -> {new_state}")
        self.state = new_state
        
        if self.state == 'FORWARD':
            self.state_start_time = rospy.Time.now()
            rospy.sleep(0.5)
            rospy.loginfo(f"#--- LADO {self.side_counter + 1}: Seguindo em linha reta... ---#")
            # TODO: call manual mode service
        
        elif self.state == 'TURN':
            self.stop_the_boat()
            self.target_yaw = self.normalize_angle(self.current_yaw - math.radians(90))
            # TODO: call manual mode service

        elif self.state == 'DONE':
            rospy.loginfo("Percurso do quadrado finalizado!")
            self.stop_the_boat()

    def run(self): # TODO: set while condition
        if self.current_yaw is None:
            return

        if self.state == 'FORWARD':
            self.set_thrusters(self.THRUST_FORWARD, self.THRUST_FORWARD)            
            if rospy.Time.now() - self.state_start_time >= self.SIDE_DURATION:
                self.side_counter += 1
                if self.side_counter >= 4:
                    self.change_state('DONE')
                else:
                    self.change_state('TURN')

        elif self.state == 'TURN':
            error_rad = self.normalize_angle(self.target_yaw - self.current_yaw)
            error_degrees = math.degrees(error_rad)
            rospy.loginfo_throttle(1, f"Alvo: {math.degrees(self.target_yaw):.1f}, Atual: {math.degrees(self.current_yaw):.1f}, Erro: {error_degrees:.1f}°")
            if abs(error_degrees) <= self.DEGREE_RANGE:
                self.change_state('FORWARD')
            else:
                self.set_thrusters(self.THRUST_TURN, -self.THRUST_TURN)
        
        elif self.state == 'DONE':
            self.stop_the_boat()





        '''
        while not self.imu_received and not rospy.is_shutdown(): ###
            rospy.loginfo_once("Aguardando IMU...") ###
            rospy.sleep(0.5) ###

        if rospy.is_shutdown(): ###
            return ###
        
        rospy.loginfo("Percorrendo o quadrado!")
        
        for i in range(4):
            rospy.loginfo(f"#--- LADO {i+1}: Seguindo em linha reta... ---#")
            start_time = rospy.Time.now()
            rate = rospy.Rate(20) ###
            while rospy.Time.now() - start_time < rospy.Duration(self.SIDE_DURATION) and not rospy.is_shutdown(): ##
                self.set_thrusters(self.THRUST_FORWARD, self.THRUST_FORWARD)
                rate.sleep()
            
            if i == 3:
                break

            self.stop_the_boat()
            rospy.loginfo("Parada completa. Movendo para próxima curva...")
            rospy.loginfo(f"#--- CURVA {i+1}. ---#")
            target_angle = self.normalize_angle(self.current_yaw  - math.radians(90))
            self.turn(target_angle)
        
        rospy.loginfo("Percurso do quadrado finalizado!")
        self.stop_the_boat()
        '''

def main():
    rospy.init_node('square_node', anonymous=True)
    rospy.loginfo("Nó 'square_node' inicializado.")
    controller = SquareNode() ### set rate on main
    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            controller.state()
            rate.sleep()
    finally:
        rospy.loginfo('Parando o barco...')
        controller.stop_the_boat()
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido (Ctrl+C).")
        pass
