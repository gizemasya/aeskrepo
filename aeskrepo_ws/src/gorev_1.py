#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import std_msgs
import math
import tf
import rospy
import numpy as np

import math
from math import sin, cos, pi

class Drone:

    def __init__(self):

        self.sürü_merkezi = []
        self.formasyon_list = []

        self.pos_x = float()
        self.pos_y = float()   
        self.pos_z = float()

        self.first_run_take_off = True
        self.take_off_x = float()
        self.take_off_y = float()

        self.rate = rospy.Rate(10)

        self.cmd_x = float()
        self.cmd_y = float()
        self.cmd_z = float()
        
        self.position = [self.pos_x, self.pos_y, self.pos_z] 

        self.dt = 1/10

        rospy.Subscriber("/firefly1/odometry_sensor1/odometry" , Odometry, self.callback)
        rospy.Subscriber("/firefly2/odometry_sensor1/odometry" , Odometry, self.callback)
        rospy.Subscriber("/firefly3/odometry_sensor1/odometry" , Odometry, self.callback)

    def callback(self, msg):

        self.pos_x = msg.pose.pose.position.x  
        self.pos_y = msg.pose.pose.position.y        
        self.pos_z = msg.pose.pose.position.z
        #print(self.pos_x, self.pos_y, self.pos_z)

    def position_control (self, desired_x_to_go, desired_y_to_go, desired_z_to_go, desired_yaw):

        firefly1_command_publisher = rospy.Publisher("/firefly1/command/trajectory", MultiDOFJointTrajectory, queue_size=10,)
        firefly2_command_publisher = rospy.Publisher("/firefly2/command/trajectory", MultiDOFJointTrajectory, queue_size=10,)
        firefly3_command_publisher = rospy.Publisher("/firefly3/command/trajectory", MultiDOFJointTrajectory, queue_size=10,)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(desired_yaw))  # roll,yaw pitch için
        traj = MultiDOFJointTrajectory()  # kontrolcüye gönderilecek mesaj

        # mesaja istenen parametrelerin aktarılması (header)
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time()
        header.frame_id = "frame"
        traj.joint_names.append("base_link")
        traj.header = header

        # istenen nokta için dönüşümler
        transforms = Transform(translation=Point(desired_x_to_go, desired_y_to_go, desired_z_to_go),rotation=Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),)
        velocities = Twist()
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Time(2))
        traj.points.append(point)

        firefly1_command_publisher.publish(traj)
        firefly2_command_publisher.publish(traj)
        firefly3_command_publisher.publish(traj)


    def velocity_control(self, v_setpoint_x, v_setpoint_y, v_setpoint_z):
        # Hızın zaman ile çarpılmasıyla gidilecek yol bulunur ve ilk konuma eklenir. Böylece son konum bulunur. 
        self.cmd_x = self.pos_x + v_setpoint_x * self.dt
        self.cmd_y = self.pos_y + v_setpoint_y * self.dt
        self.cmd_z = self.pos_z + v_setpoint_z * self.dt

        self.position_control(self.cmd_x, self.cmd_y, self.cmd_z, 0)

    def sürü_merkezi_kontrolü(self, merkez_x, merkez_y, merkez_z):

        while merkez_x < 3:

            self.sürü_merkezi = [merkez_x, merkez_y, merkez_z]
            self.velocity_control(merkez_x, merkez_y, merkez_z)

            #self.formasyon_list = [self.sürü_merkezi, for_1, for_2, for_3]

            merkez_x = merkez_x + 0.5
            merkez_y = merkez_y + 0.5
            merkez_z = merkez_z + 0.5
            
    def take_off(self, alt):
        while self.pos_z < alt:
            print(self.pos_x, self.pos_z)
            self.velocity_control(0,0,3)
            self.rate.sleep()
            

    def landing(self):
        while self.pos_z > 0.1:
            self.velocity_control(0,0,-3)
            self.rate.sleep()

rospy.init_node('gcs')
crazyflie = Drone()

'''drone_1 = Drone()
drone_2 = Drone()
drone_3 = Drone()'''   

if __name__ == '__main__':
    while True:
        crazyflie.take_off(3)
        crazyflie.sürü_merkezi_kontrolü(0, 0, 0) #mutlak sıfır noktası
        #crazyflie.velocity_control(3, 0, 3)
        #crazyflie.take_off(3)
        #crazyflie.landing()

        '''drone_1.velocity_control(3, 0, 0)
        drone_2.velocity_control(0, 3, 0)
        drone_3.velocity_control(0, 0, 3)
        
        drone_1.take_off(3)
        drone_2.take_off(3)
        drone_3.take_off(3)

        drone_1.landing()
        drone_2.landing()
        drone_3.landing()'''