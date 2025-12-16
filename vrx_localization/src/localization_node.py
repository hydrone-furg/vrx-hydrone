#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPoseStamped

GPS_TOPIC = '/wamv/sensors/gps/gps/fix'
IMU_TOPIC = '/wamv/sensors/imu/imu/data'

class LocalizationNode:
    def __init__(self):
        self.latest_gps = None
        self.latest_imu = None

        # Subscriber
        self.gps_subscriber = rospy.Subscriber(GPS_TOPIC, NavSatFix, self.gps_callback)
        self.imu_subscriber = rospy.Subscriber(IMU_TOPIC, Imu, self.imu_callback)

        # Publisher
        self.geopose_publisher = rospy.Publisher('/wamv/geopose', GeoPoseStamped, queue_size=10)

        rospy.loginfo('Nó de localização iniciado.')
        rospy.loginfo('Aguardando GPS e IMU...')

    def gps_callback(self, msg):
        if self.latest_gps is None:
            rospy.loginfo('Mensagem de GPS recebido.')
        self.latest_gps = msg

    def imu_callback(self, msg):
        if self.latest_imu is None:
            rospy.loginfo('Mensagem de IMU recebido.')
        self.latest_imu = msg

    def run(self):
        if self.latest_gps is not None and self.latest_imu is not None:
            geo_pose_msg = GeoPoseStamped()

            # Header:
            geo_pose_msg.header.stamp = rospy.Time.now()
            geo_pose_msg.header.frame_id = 'map'

            # Pose:
            geo_pose_msg.pose.position.latitude = self.latest_gps.latitude
            geo_pose_msg.pose.position.longitude = self.latest_gps.longitude
            geo_pose_msg.pose.position.altitude = self.latest_gps.altitude

            # > orientation
            geo_pose_msg.pose.orientation = self.latest_imu.orientation

            # > publish
            self.geopose_publisher.publish(geo_pose_msg)

def main():
    rospy.init_node('wamv_localization_node', anonymous=True)
    localization = LocalizationNode()
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            localization.run()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Programa interrompido (Ctrl+C).")
        pass

if __name__ == '__main__':
    main()