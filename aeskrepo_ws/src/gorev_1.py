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

        self.formations_x = []
        self.formations_y = []
        self.formations_z = []
        self.new_formations_x = []
        self.new_formations_y = []
        self.new_formations_z = []



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

    def swarm_center_control(self, cx, cy, cz):
        # sürü merkez noktasının koordinatları, ilk başta hepsi 0.
        self.swarm_center_x = cx
        self.swarm_center_y = cy
        self.swarm_center_z = cz

        # formasyon noktalarının koordinatları, istenilen değerlere eşitlenecekler.
        x_1, x_2, x_3, y_1, y_2, y_3, z_1, z_2, z_3 = float()

        # sürü merkezi ve tüm formasyonların olduğu liste
        self.formations_x.append(cx, x_1, x_2, x_3)
        self.formations_y.append(cy, y_1, y_2, y_3)
        self.formations_z.append(cz, z_1, z_2, z_3)

        # istenilen final noktasına kadar merkez ve formasyonların konumu 0.5'er 0.5'er artar.
        while cx < final_pos_x:
            self.new_formations_x = [x + 0.5 for x in self.formations_x]

        while cy < final_pos_y:
            self.new_formations_y = [y + 0.5 for y in self.formations_y]

        while cz < final_pos_z:
            self.new_formations_z = [z + 0.5 for z in self.formations_z]

        # self.velocity_control() vs yazcaz

   
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
        final_pos_x = 0
        final_pos_y = 0
        final_pos_z = 5

        crazyflie.take_off(3)
        crazyflie.swarm_center_control(0, 0, 0) #mutlak sıfır noktası
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
