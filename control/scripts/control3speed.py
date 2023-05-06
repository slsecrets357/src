#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String, Byte
from utils.msg import Lane, Sign, localisation, IMU, encoder
# from utils.srv import get_direction, dotted, nav
import time
import math

import cv2
import os
import json
import threading
import argparse
import socket

import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from trackmap import track_map

class StateMachine():
    #initialization
    def __init__(self, simulation = False, planned_path = "/paths/plannedpathspeed.json", custom_path = False, localisation = False):
        rospy.init_node('control_node', anonymous=True)
        self.rate = rospy.Rate(25)
        self.dt = 1/25 #for PID

        self.localise_before_decision = localisation
        if localisation:
            print("localisation on")
        self.msg = String()
        self.history = None

        #simulation
        self.simulation = simulation
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
        
        self.publish_cmd_vel = self.publish_cmd_vel_real
        self.class_names = ['oneway', 'highwayentrance', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayexit', 'priority',
            'lights','block','pedestrian','car','others','nothing']
        self.min_sizes = [25,25,40,50,40,35,30,25,25,150,75,72,125]
        self.max_sizes = [100,75,125,100,125,125,70,75,100,350,170,250,300]

        # get initial yaw from IMU
        self.initialYaw = 0
        #launch sensors at 0 to remove this
        #or get the yaw offset from 0 after run
        # while self.initialYaw==0:
        #     imu = rospy.wait_for_message("/automobile/IMU",IMU)
        #     self.initialYaw = imu.yaw
        #     print("initialYaw: "+str(self.initialYaw))
        print("Real mode")
        self.odomRatio = 0.0066
        self.process_yaw = self.process_yaw_real
        self.left_trajectory = self.left_trajectory_real
        self.right_trajectory = self.right_trajectory_real
        self.parallelParkAngle = 35
        self.initializationTime = 1
        self.maxspeed = 0.17
        file = open(os.path.dirname(os.path.realpath(__file__))+'/PID.json', 'r')
        
        # self.x = 0.82
        # self.y = 14.91
        # self.yaw = 1.5707
        # self.plan_path(custom_path, planned_path)
        
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        # self.decisions = [2,3,6,0,4]
        # self.decisions = [3,5,1]
        # self.decisions = [2,2,2,2,2,2,2,2,2]
        self.decisions = [2, 1, 9, 14, 2, 1, 0, 1, 2]
        # self.decisions = [0]
        self.decisionsI = 0
        self.full_path = ['test','test','test','test','test','test','test','test','test','test','test','test','test']
        self.planned_path = ['test1']
        #states
        self.states = ['Lane Following', "Approaching Intersection", "Stopping at Intersection", 
                       "Intersection Maneuvering", "Approaching Crosswalk", "Pedestrian", "Highway",
                       "overtaking", "Roundabout", "Parking", "Initial", "Parked", "Curvedpath"] #13 states
        self.confidence_thresh = [0.8, 0.8, 0.83, 0.8, 0.8, 0.83, 0.8, 0.8, 0.83, 0.83, 0.85, 0.853]
        self.state = 10 #initial
        if self.history is None:
            self.history = 0
        self.highwaySpeed = self.maxspeed*1.33
        self.highwayRight = True
        self.highwaySide = 1 #1:right, -1:left
        self.laneOvertakeAngle = np.pi*0.175
        self.laneOvertakeCD = 2
        self.overtakeDuration = 1
        self.doneCurvedPath = False
        # self.overtake = False

        #sign
        # self.class_names = ['oneway', 'highwayentrance', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayexit', 'priority',
        #         'lights','block','pedestrian','car','others','nothing']
        # self.min_sizes = [25,25,40,50,40,35,30,25,25,130,75,72,130]
        # self.max_sizes = [100,75,125,100,120,125,70,75,100,350,170,250,300]
        self.center = -1
        self.image_center = 320
        self.ArrivedAtStopline = False
        self.detected_objects = []
        self.numObj = -1
        self.box1 = []
        self.box2 = []
        self.box3 = []
        self.confidence = []

        #pose (will get them from localisation)
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
        self.last = 0
        self.pl = 320 # previous lane center

        #timers
        self.timer = None
        self.timerpid = rospy.Time.now()
        self.timerodom = rospy.Time.now()
        self.timerPedestrian = None
        self.timerP = None
        self.timerO = None
        self.timer1 = None
        self.timer2 = None
        self.exitCD = rospy.Time.now()
        self.flag1 = True

        #intersection & parking
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.intersectionStop = False
        self.intersectionDecision = -1
        self.parkingDecision = -1
        self.exitDecision = -1
        self.rdbDecision = -1
        self.decisionList = ["left","straight","right","parkF","parkP",
        "exitparkL","exitparkR","exitparkP","enterhwLeft","enterhwStright","rdb",
        "exitrdbE","exitrdbS","exitrdbW","curvedpath"]
        self.doneManeuvering = False
        self.doneParking = False
        self.initialPoints = None
        self.intersectionState = 0
        self.trajectory = None
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
        self.offset = 0.3

        # Subscribe to topics
        self.lane_sub = rospy.Subscriber('lane', Lane, self.lane_callback, queue_size=3)
        # self.localization_sub = message_filters.Subscriber("/automobile/localisation", localisation, queue_size=3)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
        self.encoder_sub = rospy.Subscriber("/automobile/encoder", encoder, self.encoder_callback, queue_size=3)

        self.parkAdjust = True #adjust flag for forward/backward

        #size of detected objects
        self.parksize = 0
        self.parkSecond = False
        self.carsize = 0

        #flag for light, highway, curvedpath and roadblock
        self.light = False
        self.hw = False
        self.cp = False
        self.roadblock = False

        self.rdbExitYaw = 0
        # self.rdbTransf = 0
        # self.carBlockSem = -1
        self.toggle = 0
        # self.t1 = time.time()
        self.adjustYawError = 0.2 #yaw adjust for intersection maneuvering
        self.overtaking_angle = self.yaw

        #stop at shutdown
        def shutdown():
            self.lane_sub.unregister()
            if self.simulation:
                pub = rospy.Publisher("/automobile/command", String, queue_size=3)
                msg = String()
                msg2 = String()
                # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
                msg.data = '{"action":"1","speed":'+str(0.0)+'}'
                msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
                for haha in range(10):
                    pub.publish(msg2)
                    pub.publish(msg)
                    self.rate.sleep()
            else:
                msg = String()
                msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
                for haha in range(20):
                    self._write(msg)
                    self.rate.sleep()
        
        rospy.on_shutdown(shutdown)

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

    def process_yaw_real(self, yaw):
        if yaw>0:
            newYaw = -((yaw-self.initialYaw)*3.14159/180)
            self.yaw = newYaw if newYaw>0 else (6.2831853+newYaw)

    #callback functions
    def lane_callback(self,lane):
        self.center = lane.center
        self.ArrivedAtStopline = lane.stopline
        # if there's a big shift in lane center: ignore due to delay
        if abs(self.center-self.pl)>250:
            self.center = self.pl
        # ignore one center measurement when we don't detect
        if self.center==320:
            c = self.center
            self.center = self.pl
            self.pl = c
        else:
            self.pl = self.center
        # print("time: ", time.time()-self.t1)
        # self.t1 = time.time()
        act = self.action()
        if int(act)==1:
            print(f"-----transitioning to '{self.states[self.state]}'-----")
            if self.state==0:
                print("Speed is at "+str(self.maxspeed)+"m/s")
    def encoder_callback(self,encoder):
        self.velocity = encoder.speed
    def imu_callback(self,imu):
        self.process_yaw(imu.yaw)

    #state machine
    def action(self):
        if self.state==0: #lane following
            return self.lanefollow()
        elif self.state == 1: #Approaching Intersection
            return self.approachInt()
        elif self.state == 2: #Stopping at Intersection
            return self.stopInt()
        elif self.state == 3: #Intersection Maneuvering
            return self.maneuverInt()
        elif self.state == 4: #Approaching Crosswalk
            return self.approachCrosswalk()
        elif self.state == 5: #Pedestrian
            return self.stopPedestrian()
        elif self.state == 6: #Highway
            return self.highway()
        elif self.state == 7: #overtake
            # if self.history == 6:
            #     return self.highway_overtake()
            # else:
            return self.lane_overtake()
        elif self.state == 8: #Roundabout
            return self.roundabout()
        elif self.state == 9: #Parking
            if self.timerP is None:
                self.timerP = rospy.Time.now() + rospy.Duration(1.57) # stop before parking
                print("prepare to park")
            elif rospy.Time.now() >= self.timerP:
                return self.park()
            self.idle()
            return 0
        elif self.state == 10: #initialization state
            # self.localise()
            if self.timer is None:
                print("Initializing controller...")
                self.timer = rospy.Time.now() + rospy.Duration(self.initializationTime)
            if rospy.Time.now() >= self.timer:
                print("done initializing.")
                self.timer = None
                self.state = self.history
                return 1
            else:
                if self.toggle == 0:
                    self.toggle = 1
                    if self.simulation:
                        self.idle()
                    else:
                        self.msg.data = '{"action":"4","activate": true}'
                elif self.toggle == 1: 
                    self.toggle = 2
                    self.idle()
                elif self.toggle == 2:
                    self.toggle = 0
                    if self.simulation:
                        self.idle()
                    else:
                        self.msg.data = '{"action":"5","activate": true}'
                if not self.simulation:
                    self._write(self.msg)
                return 0
        elif self.state == 11: #parked
            # if self.decisionsI >= len(self.decisions):
            #     self.idle()
            #     self.idle()
            #     self.idle()
            #     rospy.signal_shutdown("Exit")
            #     return 0
            # else:
                if self.timerP is None:
                    self.timerP = rospy.Time.now() + rospy.Duration(1.57) # stop before parking
                    print("prepare to exit")
                elif rospy.Time.now() >= self.timerP:
                    return self.exitPark()
                self.idle()
                return 0
        elif self.state == 12: #Curvedpath
            return self.curvedpath()
    
    #actions
    def lanefollow(self):
        # Determine the steering angle based on the center and publish the steering command
        if self.ArrivedAtStopline:
            print("signless intersection detected... -> state 3")
            self.doneManeuvering = False #set to false before entering state 3
            self.state = 3
            self.timer0 = None
            self.timer2 = None
            return 1
        self.publish_cmd_vel(self.get_steering_angle())
        return 0
    
    def maneuverInt(self):
        if self.doneManeuvering:
            print("done intersection maneuvering.")
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            if self.hw:
                print("entering highway -> 6")
                self.state = 6
            elif self.cp:
                self.state = 8
            else:
                self.state = 0 #go back to lane following
            self.hw = False
            self.cp = False
            self.initialPoints = None #reset initial points
            self.pl = 320
            self.adjustYawError = 0.2
            return 1
        elif self.intersectionDecision < 0:
            if self.decisionsI >= 4:
                self.maxspeed = 0.375
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.intersectionDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            # if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
            #     self.planned_path.pop(0)
            if self.intersectionDecision == 8:
                self.intersectionDecision = 0
                self.hw = True
            elif self.intersectionDecision == 9:
                self.intersectionDecision = 1
                self.hw = True
            elif self.intersectionDecision == 10:
                print("entering roundabout -> 8")
                self.intersectionDecision = -1 #reset
                self.state = 8
                return 1
            if self.intersectionDecision == 0: #left
                self.trajectory = self.left_trajectory
            elif self.intersectionDecision == 1: #straight
                self.trajectory = self.straight_trajectory
            elif self.intersectionDecision == 2: #right
                self.trajectory = self.right_trajectory
                if self.cp:
                    self.cp = False
            else:
                # self.plan_new_path()
                self.decisionsI -= 1
                self.doneManeuvering = True
                print("self.intersectionDecision id wrong: ",self.intersectionDecision)
                return 0
            print("intersection decision: going " + self.decisionList[self.intersectionDecision])
        if self.initialPoints is None:
            self.set_current_angle()
            # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.intersectionState = 0 #adjusting angle:0, trajectory following:1, adjusting angle2: 2..
            self.adjustYawError = 0.2 if self.intersectionDecision!=1 else 0.03
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        # print("position: ",x,y)
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, curAngle, error: ", self.yaw, self.currentAngle, error)
            if abs(error) <= self.adjustYawError:
                self.intersectionState+=1 #done adjusting
                # print("done adjusting angle. Transitioning to trajectory following")
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed) if self.intersectionDecision != 2 else self.publish_cmd_vel(21, self.maxspeed)
                return 0
        elif self.intersectionState==1: #trajectory following
            desiredY = self.trajectory(x) if x<self.offsets_x[self.intersectionDecision] else self.offsets_y[self.intersectionDecision]
            error = y - desiredY
            # print("x, y_error: ",x,abs(error))
            # if self.yaw >= 5.73:
            #     self.yaw -= 
            print("yaw, self.destinationAngle: ", self.yaw, self.destinationAngle)
            arrived = abs(self.yaw-self.destinationAngle) <= 0.375 or abs(self.yaw-self.destinationAngle) >= 4.
            if self.intersectionDecision == 1:
                arrived = arrived and abs(x)>=1 and abs(y-self.offsets_y[self.intersectionDecision]) < 0.2
            # if self.roadblock:
            #     arrived = arrived and error < 0.2
            # print("yaw_error: ")
            # print(str(self.yaw-self.destinationAngle))
            if arrived:
                # print("trajectory done.")
                self.doneManeuvering = True
                self.last_error2 = 0 #reset pid errors
                self.error_sum2 = 0
                return 0
            # steering_angle = self.pid2(error)
            # print("steering: ",steering_angle)
            # print("x, y, desiredY, angle, steer: ", x, y, desiredY, self.yaw, steering_angle*180/3.14159)
            self.publish_cmd_vel(self.pid2(error), self.maxspeed)
            return 0
    
    def highway(self):
        if self.decisionsI < len(self.decisions):
            if self.decisions[self.decisionsI] == 14 and abs(self.yaw-0.15) <= 0.05: #tune this
                self.doneManeuvering = False
                self.timer0 = None
                self.flag1 = True
                self.highwaySpeed = self.maxspeed*1.33
                self.state = 12
                self.highwaySide = 1
                return 1
        if self.ArrivedAtStopline:
            self.doneManeuvering = False #set to false before entering state 3
            self.timer0 = None
            self.flag1 = True
            self.highwaySpeed = self.maxspeed*1.33
            self.state = 3
            self.highwaySide = 1
            return 1
        self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed*1.33)
        return 0

    def curvedpath(self):
        if self.doneManeuvering:
            print("done curvedpath maneuvering.")
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            self.initialPoints = None #reset initial points
            self.pl = 320
            self.state = 0
            self.cp = True
            return 1
        elif self.intersectionDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            self.intersectionDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            # if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
            #     self.planned_path.pop(0)
            if self.intersectionDecision == 14:
                pass
            else:
                self.doneManeuvering = True
                print("self.intersectionDecision id wrong: ",self.intersectionDecision)
                return 0
            print("highway decision: going " + self.decisionList[self.intersectionDecision])
        if self.initialPoints is None:
            self.set_current_angle()
            # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            self.offset = 1.5 #tune this
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.intersectionState = 0
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        # print("position: ",x,y)
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, curAngle, error: ", self.yaw, self.currentAngle, error)
            if x >= self.offset:
                print("trajectory done.")
                self.doneManeuvering = True
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed)
                return 0
    
    #controller functions
    def idle(self):
        # self.cmd_vel_pub(0.0, 0.0)
        # self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        # self.cmd_vel_pub.publish(self.msg)
        if self.simulation:
            self.msg.data = '{"action":"1","speed":'+str(0.0)+'}'
            self.msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
            self.cmd_vel_pub.publish(self.msg)
            self.cmd_vel_pub.publish(self.msg2)
        else:
            self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
            self._write(self.msg)

    #odom helper functions
    def pid(self, error):
        # self.error_sum += error * self.dt
        dt = (rospy.Time.now()-self.timerpid).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timerpid = rospy.Time.now()
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.kd * derivative #+ self.ki * self.error_sum
        self.last_error = error
        return output
    def pid2(self, error):
        # self.error_sum2 += error * self.dt
        dt = (rospy.Time.now()-self.timerpid).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timerpid = rospy.Time.now()
        derivative = (error - self.last_error2) / dt
        output = self.kp2 * error + self.kd2 * derivative #+ self.ki2 * self.error_sum2
        self.last_error2 = error
        return output
    def odometry(self):
        dt = (rospy.Time.now()-self.timerodom).to_sec()
        self.timerodom = rospy.Time.now()
        magnitude = self.velocity*dt*self.odomRatio
        self.odomX += magnitude * math.cos(self.yaw)
        self.odomY += magnitude * math.sin(self.yaw)
        # print(f"odometry: speed={self.velocity}, dt={dt}, mag={magnitude}, cos={math.cos(self.yaw)}, X={self.odomX}, Y={self.odomY}")
    def set_current_angle(self):
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.orientation = np.argmin([abs(self.yaw),abs(self.yaw-1.5708),abs((self.yaw)-3.14159),abs(self.yaw-4.71239),abs(self.yaw-6.28319)])%4
        self.currentAngle = self.orientations[self.orientation]
        if self.intersectionDecision == 0 or self.exitDecision == 5: #left
            self.destinationOrientation = self.directions[(self.orientation+1)%4]
            self.destinationAngle = self.orientations[(self.orientation+1)%4]
            return
        elif self.intersectionDecision == 1: #straight
            self.destinationOrientation = self.orientation
            self.destinationAngle = self.currentAngle
            return
        elif self.intersectionDecision == 2 or self.parkingDecision == 3 or self.exitDecision == 6: #right
            self.destinationOrientation = self.directions[(self.orientation-1)%4]
            self.destinationAngle = self.orientations[(self.orientation-1)%4]
            return
        elif self.parkingDecision == 4:
            self.destinationOrientation = self.directions[(self.orientation)%4]
            self.destinationAngle = self.orientations[(self.orientation)%4]
            return

    #trajectories
    def straight_trajectory(self, x):
        return 0
    def left_trajectory_real(self, x):
        return math.exp(2.0*x-7.53)
    def right_trajectory_real(self, x):
        return -math.exp(4*x-3.75)
    def left_exit_trajectory_real(self, x):
        return math.exp(4*x+2)
    def right_exit_trajectory_real(self, x):
        return -math.exp(4*x-1.65)
    def leftpark_trajectory(self, x):
        return math.exp(3.57*x-4.2) #real dimensions
    def left_trajectory_sim(self, x):
        return math.exp(3.57*x-4.53)
    def right_trajectory_sim(self, x):
        return -math.exp(3.75*x-3.33)
    def left_exit_trajectory_sim(self, x):
        return math.exp(3*x-0.75)
    def right_exit_trajectory_sim(self, x):
        return -math.exp(3.75*x-2.53)
    def rdb_trajectory(self, x, t):
        u = 0.5-math.pow(x-0.71,2)
        u = np.clip(u,0,255)
        yaw = self.yaw+t if self.yaw+t>0 else (6.2831853+self.yaw+t)
        return -math.sqrt(u)+0.25 if yaw<np.pi/2 or yaw>3*np.pi/2 else math.sqrt(u)+0.25

    #others
    def get_steering_angle(self,offset=35):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in radians
        """
        # Calculate the steering angle in radians
        self.dt = (rospy.Time.now()-self.timerpid).to_sec()
        self.timerpid = rospy.Time.now()
        error = (self.center + offset - self.image_center)
        try:
            d_error = (error-self.last)/self.dt
        except:
            return 0
        self.last = error
        steering_angle = (error*self.p+d_error*self.d)
        return steering_angle
    def publish_cmd_vel_real(self, steering_angle, velocity = None, clip = True):
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
    def publish_cmd_vel_sim(self, steering_angle, velocity = None, clip = True):
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle*180/np.pi, -22.9, 22.9)
        self.msg.data = '{"action":"1","speed":'+str(velocity)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(float(steering_angle))+'}'
        # print(self.msg.data)
        # print(self.msg2.data)
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='State Machine for Robot Control.')
    parser.add_argument("--simulation", type=str, default=True, help="Run the robot in simulation or real life")
    parser.add_argument("--path", type=str, default="/paths/path.json", help="Planned path")
    parser.add_argument("--custom", type=str, default=False, help="Custom path")
    parser.add_argument("--localisation", type=str, default=False, help="localisation enabled")
    # args, unknown = parser.parse_known_args()
    args = parser.parse_args(rospy.myargv()[1:])
    s = args.simulation=="True"
    c = args.custom=="True"
    l = args.localisation=="True"
    node = StateMachine(simulation=s,planned_path=args.path,custom_path=c,localisation=l)
    rospy.spin()