#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String
from utils.msg import localisation, IMU, encoder
# from pynput import keyboard
# import time
import math
import os
import json

class Odomtest():
    def __init__(self):
        # serialNODE
        from messageconverter import MessageConverter
        import serial
        devFile = '/dev/ttyACM0'
        
        # comm init
        self.serialCom = serial.Serial(devFile,19200,timeout=1)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()
        
        # message converted init
        self.messageConverter = MessageConverter()

        #pose
        self.x = 0.82
        self.y = 14.91
        self.yaw = 1.5707
        self.speed = 0
        self.odomX = 0
        self.odomY = 0

        #PID
        #for initial angle adjustment
        self.error_sum = 0
        self.last_error = 0
        #for goal points
        self.error_sum2 = 0
        self.last_error2 = 0
        # Load PIDs data
        file = open(os.path.dirname(os.path.realpath(__file__))+'/PID.json', 'r')
        data = json.load(file)
        print("PIDs params:")
        print(data)
        self.p = data.get('p')
        self.d = data.get('d')
        self.i = data.get('i')
        self.kp = data.get('kp')
        self.kd = data.get('kd')
        self.ki = data.get('ki')
        self.kp2 = data.get('kp2')
        self.kd2 = data.get('kd2')
        self.ki2 = data.get('ki2')

        #steering
        self.msg = String()
        self.msg2 = String()
        self.maxspeed = 0.13
        self.last = 0

        self.toggle = 0

        #constants
        self.orientation = 1 #0,1,2,3=east,north,west,south
        self.directions = ["east", "north", "west", "south"]
        self.orientations = np.array([0,1.5708,3.14159,4.7124]) #0, pi/2, pi, 3pi/2
        self.left_offset_x = 1.20
        self.left_offset_y = 0.82
        self.right_offset_x = 0.80
        self.right_offset_y = 0.573
        self.velocity = self.maxspeed
        self.offsets_x = np.array([self.left_offset_x, self.left_offset_x*1.2, self.right_offset_x])
        self.offsets_y = np.array([self.left_offset_y, 0, self.right_offset_y])
        self.rotation_matrices = np.array([[[1,0],[0,1]],[[0,-1],[1,0]],[[-1,0],[0,-1]],[[0,1],[-1,0]]]) #E,N,W,S

        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.timer4 = rospy.Time.now()
        self.timer5 = rospy.Time.now()
        self.timer6 = rospy.Time.now()
        self.odomTimer = rospy.Time.now()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=3)
        self.rate = rospy.Rate(30)
        self.dt = 1/50 #for PID

        self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
        self.encoder_sub = rospy.Subscriber("/automobile/encoder", encoder, self.encoder_callback, queue_size=3)

        # get initial yaw from IMU
        self.initialYaw = 0
        while self.initialYaw==0:
            imu = rospy.wait_for_message("/automobile/IMU",IMU)
            self.initialYaw = imu.yaw
            print("initialYaw: "+str(self.initialYaw))

        #stop at shutdown
        def shutdown():
            self.imu_sub.unregister()
            self.encoder_sub.unregister()
            msg = String()
            msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
            for haha in range(20):
                self._write(msg)
                self.rate.sleep()
        
        rospy.on_shutdown(shutdown)

        #enable encoder at the start to get messages from automobile/encoder
        self.msg.data = '{"action":"5","activate": true}'
        self._write(self.msg)
        self._write(self.msg)
        self._write(self.msg)

        self.timerP = None

    def _write(self, msg):
        """ Represents the writing activity on the the serial.
        """
        # print(msg.data)
        command = json.loads(msg.data)
        command_msg = self.messageConverter.get_command(**command)
        # print(command_msg)
        self.serialCom.write(command_msg.encode('ascii'))
        # buffer_size = self.serialCom.out_waiting
        # print("Buffer size:", buffer_size)
    
    def encoder_callback(self,encoder):
        self.velocity = encoder.speed
    def imu_callback(self,imu):
        if self.timer == None:
            print("initializing...")
            self.timer = rospy.Time.now() + rospy.Duration(2)
        if rospy.Time.now() <= self.timer:
            if self.toggle == 0:
                self.toggle == 1
                self.msg.data = '{"action":"5","activate": true}'
                self._write(self.msg)
            else:
                self.toggle == 0
                self.msg.data = '{"action":"4","activate": true}'
                self._write(self.msg)
            return

        self.dt = (rospy.Time.now()-self.timer6).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timer6 = rospy.Time.now()

        # self.x = localization.posA
        # self.y = 15.0-localization.posB
        if imu.yaw!=0:
            yaw = -((imu.yaw-self.initialYaw)*3.14159/180)
            self.yaw = yaw if yaw>0 else (6.2831853+yaw)
        self.velocity = encoder.speed
        # print("x,y,yaw,velocity: ", self.x, self.y, self.yaw, self.velocity)
        self.orientation = np.argmin([abs(self.yaw),abs(self.yaw-1.5708),abs((self.yaw)-3.14159),abs(self.yaw-4.71239),abs(self.yaw-6.28319)])%4
        # self.orientation = 0
        self.currentAngle = self.orientations[self.orientation]
        error = self.yaw-self.currentAngle
        if self.yaw>=5.73: #subtract 2pi to get error between -pi and pi
            error-=6.28
        # print("yaw, goal, error: ", self.yaw, self.currentAngle, error)
        if abs(error) <= 0.05:
            # print("done adjusting angle.")
            self.idle()
            return
        else:
            self.publish_cmd_vel(self.pid(error), self.maxspeed*0.7)

    def idle(self):
        # self.cmd_vel_pub(0.0, 0.0)
        self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        self._write(self.msg)

    #helper functions
    def pid(self, error):
        # self.error_sum += error * self.dt
        dt = (rospy.Time.now()-self.timer4).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timer4 = rospy.Time.now()
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.kd * derivative #+ self.ki * self.error_sum
        self.last_error = error
        return output
    def pid2(self, error):
        # self.error_sum2 += error * self.dt
        dt = (rospy.Time.now()-self.timer5).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timer4 = rospy.Time.now()
        derivative = (error - self.last_error2) / dt
        output = self.kp2 * error + self.kd2 * derivative #+ self.ki2 * self.error_sum2
        self.last_error2 = error
        return output
    def odometry(self):
        dt = (rospy.Time.now()-self.odomTimer).to_sec()
        self.odomTimer = rospy.Time.now()
        magnitude = self.velocity*dt*0.0066
        self.odomX += magnitude * math.cos(self.yaw)
        self.odomY += magnitude * math.sin(self.yaw)
        # print(f"odometry: speed={self.velocity}, dt={dt}, mag={magnitude}, cos={math.cos(self.yaw)}, X={self.odomX}, Y={self.odomY}")
    def get_steering_angle(self):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in radians
        """
        # Calculate the steering angle in radians
        image_center = 640 / 2 
        error = (self.center - image_center)
        d_error = (error-self.last)/self.dt
        self.last = error
        steering_angle = (error*self.p+d_error*self.d)
        return steering_angle
    def publish_cmd_vel(self, steering_angle, velocity = None, clip = True):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle*180/np.pi, -22.9, 22.9)
        if self.toggle == 0:
            self.toggle = 1
            self.msg.data = '{"action":"1","speed":'+str(float("{:.4f}".format(velocity)))+'}'
        else:
            self.toggle = 0
            self.msg.data = '{"action":"2","steerAngle":'+str(float("{:.2f}".format(steering_angle)))+'}'
        self._write(self.msg)

if __name__ == '__main__':
    node = Odomtest()
    while not rospy.is_shutdown():
        node.rate.sleep()
        rospy.spin()