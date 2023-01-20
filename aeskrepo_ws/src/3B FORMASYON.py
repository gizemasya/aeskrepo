#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import std_msgs
import math
import tf
import rospy
import numpy as np
#import abu as a    #A STAR KODUNU ENTEGRE ETMEK İÇİN İMPORT EDİLİR

class Drone:   

    def __init__(self):
        self.pos_x = float()
        self.pos_y = float()   
        self.pos_z = float()

        self.first_run_take_off = True
        self.take_off_x = float()
        self.take_off_y = float()

        self.rate = rospy.Rate(5)

        self.cmd_x = float()
        self.cmd_y = float()
        self.cmd_z = float()
        
        self.position = [self.pos_x, self.pos_y, self.pos_z]

        self.dt = 1/10
        self.path = list()
        
        self.active = True
        self.rx = []
        self.ry = []
        
        


        rospy.Subscriber("/firefly1/odometry_sensor1/odometry" , Odometry, self.callback) 
        
    def callback(self, msg):

        self.pos_x = msg.pose.pose.position.x  
        self.pos_y = msg.pose.pose.position.y        
        self.pos_z = msg.pose.pose.position.z

        #print(self.pos_x, self.pos_y, self.pos_z)

    def position_control (self, desired_x_to_go, desired_y_to_go, desired_z_to_go, desired_yaw):

        firefly_command_publisher = rospy.Publisher("/firefly/command/trajectory", MultiDOFJointTrajectory, queue_size=10,)
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
        firefly_command_publisher.publish(traj)

    def velocity_control(self, v_setpoint_x, v_setpoint_y, v_setpoint_z):
        self.cmd_x = self.pos_x + v_setpoint_x * self.dt
        self.cmd_y = self.pos_y + v_setpoint_y * self.dt
        self.cmd_z = 3

        self.position_control(self.cmd_x, self.cmd_y, self.cmd_z, 0)

    def take_off(self):
        '''while self.pos_z < alt:
            print(self.pos_x, self.pos_z)
            self.velocity_control(0,0,3)
            self.rate.sleep()'''

        if self.active:
            self.abu()
            self.active = False
        
        i = 0
        
        while i < len(self.rx):
        #if self.pos_x - self.rx[i] < -0.1:
            path_error = self.rx[i]- self.pos_x
            path_errory = self.ry[i] - self.pos_y
            
            output_x = path_error/abs(path_error)
            output_y = path_errory/abs(path_errory)
            
            self.cmd_x = self.cmd_x + output_x*self.dt*1/2
            self.cmd_y = self.cmd_y + output_y*self.dt*1/2
            
            self.position_control(self.cmd_x, self.cmd_y, 6, 0)
            i = i + 1
             
            self.rate.sleep()
            
    def pid(self, hedef_x, hedef_y):
        k = 3
        deadband = 0.1
        while self.pos_x < hedef_x - deadband and self.pos_y < hedef_y - deadband:
            hiz_x = k*(hedef_x - self.pos_x)
            hiz_y= k*(hedef_y - self.pos_y)  
            self.velocity_control(hiz_x, hiz_y, 0)
            self.rate.sleep()  
            
            
    def landing(self):
        while self.pos_z > 0.1:
            self.velocity_control(0,0,-3)
            self.rate.sleep()
    
    def vector(self, ax, ay, az):
        destination = np.array([ax, ay, az])
        vector_fonk = destination - self.position
        vc = np.linalg.norm(vector_fonk)
        if vc > 0.1:
            b = vector_fonk/vc
            self.velocity_control(b[0], b[1], b[2])
            self.rate.sleep()
            
    '''def abu(self):          # A STAR ENTEGRASYONU İÇİN EKLENEN A STAR KODUNDAKİ MAİN!
        print(__file__ + " start!!")

        # start and goal position
        sx = 0.0  # [m]
        sy = 0.0  # [m]
        gx = 10.0  # [m]
        gy = 10.0  # [m]
        grid_size = 0.3  # [m]
        robot_radius = 1  # [m]

        # set obstacle positions
        ox, oy = [], []
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        
        
        a.circle(3, 3, 1, ox, oy, 1)
        

        a_star = a.AStarPlanner(ox, oy, grid_size, robot_radius)
        self.rx, self.ry = a_star.planning(sx, sy, gx, gy)
        
        
        self.rx.reverse()
        self.ry.reverse()
        
        print(self.rx, self.ry)
        
        bik = list()
        for kx, ky in zip(self.rx, self.ry):  #zip paralel döndürmek için 
            bik.append([kx, ky])
        self.path = bik
        
        return kx, ky'''

    '''def find_path(self):           #A STARDAKİ YOLU BULMAK İÇİN YAPILAN FONK
        for i in self.path :
            is_arrived = False
            while is_arrived == False: 
                b = np.array([i[0], i[1]])
                vector= np.array([self.pos_x, self.pos_y])
                error = b - vector
                error_mag = np.linalg.norm(error)
                if  error_mag <= 0.1:
                    is_arrived = True
                    
                else:
                    output = (error/np.linalg.norm(error))*3
                    output = output.tolist()
                    self.velocity_control(output[0], output[1], 3)'''
            
        

            
rospy.init_node('gcs')
crazyflie = Drone()

    #takeoff fonksiyonunu buradan çağırın.
    #crazyflie.take_off()

if __name__ == '__main__':

    '''if crazyflie.pos_z <10:
        crazyflie.position_control(10, 10, 10, 0)
    elif crazyflie.pos_z >= 10:
        crazyflie.landing()'''
    crazyflie.pid(10, 10)
    crazyflie.abu()  
    crazyflie.find_path()
    

        
    #crazyflie.landing(0, 0, 1)
    #crazyflie.vector(11,11,11)
    
        
