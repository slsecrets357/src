#!/usr/bin/env python3
import rospy
import numpy as np
from message_filters import ApproximateTimeSynchronizer
from std_msgs.msg import String
from utils.msg import Lane, Sign, localisation, IMU, encoder
# from pynput import keyboard
from utils.srv import get_direction, dotted
import message_filters
# import time
import math
import os
import json

class StateMachine():
    #initialization
    def __init__(self):
        #states
        self.states = ['Lane Following', "Approaching Intersection", "Stopping at Intersection", 
                       "Intersection Maneuvering", "Approaching Crosswalk", "Pedestrian", "Highway",
                       "Carblock", "Roundabout", "Parking", "Initial", "Parked"]
        self.state = 10

        #sign
        self.class_names = ['oneway', 'highwayexit', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayentrance', 'priority',
                'lights','block','pedestrian','car','others','nothing']
        self.min_sizes = [25,25,35,000,65,45,25,25,25,80,100,75,250]
        self.max_sizes = [50,75,70,000,90,80,50,75,50,200,150,150,400]
        self.detected_objects = []
        self.numObj = -1
        self.box1 = []
        self.box2 = []

        #pose
        self.x = 0.82
        self.y = 14.91
        self.yaw = 1.5707
        self.speed = 0
        self.odomX = 0
        self.odomY = 0

        #PID
        #for initial angle adjustment
        # self.kp = 1.57
        # self.ki = 0.005
        # self.kd = 0.3
        self.error_sum = 0
        self.last_error = 0
        #for goal points
        # self.kp2 = 1.57
        # self.ki2 = 0.00#5
        # self.kd2 = 0.0#8
        self.error_sum2 = 0
        self.last_error2 = 0

        #steering
        self.msg = String()
        self.msg2 = String()
        self.maxspeed = 0.12
        # self.p = 0.005
        # self.i = 0
        # self.d = 0.000#15
        self.last = 0

        #timers
        self.timer = None
        self.timer2 = None
        self.timer3 = None
       
        self.toggle = 0

        #intersection & parking
        self.intersectionStop = None
        self.intersectionDecision = -1 #0:left, 1:straight, 2:right
        self.intersectionDecisions = ["left", "straight", "right"]
        self.parkingDecision = -1 #0:leftParking, 1:noParking, 2:rightParking, 3:rightParallel, 4:leftParallel
        self.parkingDecisions = ["leftParking", "noParking","rightParking", "rightParallel", "leftParallel"] 
        self.doneManeuvering = False
        self.doneParking = False
        self.destination_x = None
        self.destination_y = None
        self.destination_theta = None
        self.initialPoints = None
        self.intersectionState = 0
        self.numIntersectionStates = 0
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

        #carblock
        self.carCleared = None
        self.dotted = False
        self.ArrivedAtStopline = False
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.timer4 = rospy.Time.now()
        self.timer5 = rospy.Time.now()
        self.timer6 = rospy.Time.now()
        self.odomTimer = rospy.Time.now()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=3)
        self.rate = rospy.Rate(50)
        self.dt = 1/50 #for PID

        # Create service proxy
        # self.get_dir = rospy.ServiceProxy('get_direction',get_direction)
        self.get_dotted = rospy.ServiceProxy('dotted',dotted)
        rospy.wait_for_service('dotted')
        # how to use:
        # d = self.get_dotted("dotted").dotted

        # get initial yaw from IMU
        self.initialYaw = 0
        while self.initialYaw==0:
            imu = rospy.wait_for_message("/automobile/IMU",IMU)
            self.initialYaw = imu.yaw
            print("initialYaw: "+str(self.initialYaw))

        # Subscribe to topics
        self.lane_sub = message_filters.Subscriber('lane', Lane, queue_size=3)
        self.sign_sub = message_filters.Subscriber('sign', Sign, queue_size=3)
        # self.localization_sub = message_filters.Subscriber("/automobile/localisation", localisation, queue_size=3)
        self.imu_sub = message_filters.Subscriber("/automobile/IMU", IMU, queue_size=3)
        self.encoder_sub = message_filters.Subscriber("/automobile/encoder", encoder, queue_size=3)
        self.subscribers = []
        self.subscribers.append(self.lane_sub)
        self.subscribers.append(self.sign_sub)
        # self.subscribers.append(self.localization_sub)
        self.subscribers.append(self.imu_sub)
        self.subscribers.append(self.encoder_sub)
        
        # Create an instance of TimeSynchronizer
        ts = ApproximateTimeSynchronizer(self.subscribers, queue_size=3, slop=1.15)
        ts.registerCallback(self.callback)
        
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

        #pedestrian semaphore
        self.pedestrian_sem = 20

        #stop at shutdown
        def shutdown():
            pub = rospy.Publisher("/automobile/command", String, queue_size=3)
            msg = String()
            msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
            pub.publish(msg)
            pub.publish(msg)
            pub.publish(msg)
            pub.publish(msg)
        
        rospy.on_shutdown(shutdown)

        #enable encoder at the start to get messages from automobile/encoder
        self.msg.data = '{"action":"5","activate": true}'
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg)

        self.timerP = None

        self.parkAdjust = True
        self.offset = 0.3

        #decisions
        #get them from localisation/path planner
        self.interDec=[2,0] #0:left, 1:straight, 2:right
        self.interDecI=0
        self.parkDec=[2,3] #0:leftParking, 1:noParking, 2:rightParking, 3:rightParallel, 4:leftParallel
        self.parkDecI=0
    
    #callback function
    def callback(self,lane,sign,imu,encoder):

        self.dt = (rospy.Time.now()-self.timer6).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timer6 = rospy.Time.now()

        # Perform decision making tasks

        # self.x = localization.posA
        # self.y = 15.0-localization.posB
        if imu.yaw!=0:
            yaw = -((imu.yaw-self.initialYaw)*3.14159/180)
            self.yaw = yaw if yaw>0 else (6.2831853+yaw)
        self.velocity = encoder.speed
        self.center = lane.center
        self.ArrivedAtStopline = lane.stopline
        self.detected_objects = sign.objects
        self.numObj = sign.num
        self.box1 = sign.box1
        self.box2 = sign.box2
        # print("x,y,yaw,velocity,center,stopline: ", self.x, self.y, self.yaw, self.velocity, self.center, self.ArrivedAtStopline)
        # for i in range(self.numObj):
        #     if i == 0:
        #         print(self.numObj)
        #         print(f"{self.class_names[self.detected_objects[i]]} detected! width, height: {self.box1[2]}, {self.box1[3]}")
        #     elif i == 1:
        #         print(f"{self.class_names[self.detected_objects[i]]} detected! width, height: {self.box2[2]}, {self.box2[3]}")
        #     else:
        #         print(f"{self.class_names[self.detected_objects[i]]} detected!")
        if int(self.action())==1:
            print(f"transitioning to '{self.states[self.state]}'")
            if self.state==0:
                print("Speed is at "+str(self.maxspeed)+"m/s")

    #state machine
    def action(self):
        if self.state==0: #lane following
            return self.lanefollow()
        elif self.state == 1: #Approaching Intersection
            return self.approachInt()
        elif self.state == 2: #Stopping at Intersection
            return self.stopInt()
        elif self.state == 3: #Intersection Maneuvering
            self.maneuverInt()
            # return self.maneuverIntHC()
        elif self.state == 4: #Approaching Crosswalk
            return self.approachCrosswalk()
        elif self.state == 5: #Pedestrian
            return self.stopPedestrian()
        elif self.state == 6: #Highway
            return self.highway()
        elif self.state == 7: #Carblock
            return self.carBlock()
        elif self.state == 8: #Roundabout
            self.state = 0#not implemented yet
            return 1
        elif self.state == 9: #Parking
            if self.timerP is None:
                self.timerP = rospy.Time.now() + rospy.Duration(1.57)
                print("prepare to park")
            elif rospy.Time.now() >= self.timerP:
                return self.park()
            self.idle()
            return 0
        elif self.state == 10: #initialization state
            if self.timer is None:
                print("Initializing controller...")
                self.toggle = 0
                self.timer = rospy.Time.now() + rospy.Duration(3.57)
            if rospy.Time.now() >= self.timer:
                print("done initializing.")
                self.timer = None
                self.state = 0
                return 1
            else: # stop and activated PID and encoder
                if self.toggle == 0:
                    self.toggle = 1
                    self.msg.data = '{"action":"4","activate": true}'
                elif self.toggle == 1: 
                    self.toggle = 2
                    self.idle()
                elif self.toggle == 2:
                    self.toggle = 0
                    self.msg.data = '{"action":"5","activate": true}'
                self.cmd_vel_pub.publish(self.msg)
        elif self.state == 11: #parked
            self.idle()
            self.idle()
            self.idle()
            rospy.signal_shutdown("Parked")
        return 0
    
    #actions
    def lanefollow(self):
        # Determine the steering angle based on the center and publish the steering command
        self.publish_cmd_vel(self.get_steering_angle()) 
        #transition events
        if self.ArrivedAtStopline:
            print("signless intersection detected... -> state 3")
            self.doneManeuvering = False #set to false before entering state 3
            self.state = 3
            return 1
        elif self.stop_sign_detected():
            print("stop sign detected -> state 1")
            self.intersectionStop = True
            self.state = 1
            return 1
        elif self.light_detected(): #change this to stop till light turns green
            #call service to check light color
            if self.is_green():
                print("green light detected -> state 1")
                self.intersectionStop = False
            else:
                print("red light detected -> state 1")
                self.intersectionStop = True
            self.state = 1
            return 1
        elif self.crosswalk_sign_detected():
            print("crosswalk sign detected -> state 4")
            self.state = 4
            return 1
        elif self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.pedestrian_sem = 20 #set semaphore for pedestrian
            self.history = self.state
            self.state = 5
            return 1
        elif self.highway_entrance_detected():
            print("entering highway -> 6")
            self.state = 6
            return 1
        elif self.entering_roundabout():
            print("entering roundabout -> 8")
            self.state = 8
            return 1
        elif self.parking_detected():
            print("about to park -> 9")
            self.state = 9
            return 1
        elif self.object_detected(10):
            print("Block!!!")
            self.state = 11
            return 1
        return 0
    def approachInt(self):
        #Transition events
        if self.ArrivedAtStopline:
            if self.intersectionStop:
                print("arrived at stopline. Transitioning to 'stopping at intersection'.")
                self.state = 2
                return 1
            else:
                print("arrived at stopline. Transitioning to 'intersection maneuvering' directly.")
                self.doneManeuvering = False #set to false before entering state 3
                self.state = 3
                return 1
        # Determine the steering angle based on the center publish the steering command
        self.publish_cmd_vel(self.get_steering_angle()) 
        return 0
    def stopInt(self):
        self.idle()
        #Transition events
        if self.timer is None:
            self.timer = rospy.Time.now() + rospy.Duration(3.57)
        elif rospy.Time.now() >= self.timer:
            self.timer = None
            self.doneManeuvering = False #set to false before entering state 3
            self.state = 3
            return 1
        return 0
    def maneuverInt(self):
        #go left, straight or right
        #Transition Events
        if self.doneManeuvering:
            print("done intersection maneuvering. Back to lane following...")
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            self.state = 0 #go back to lane following
            self.initialPoints = None #reset initial points
            return 1
        elif self.intersectionDecision <0:
            self.intersectionDecision = self.interDec[self.interDecI] #replace this with service call
            self.interDecI+=1
            print("intersection decision: going " + self.intersectionDecisions[self.intersectionDecision])
            if self.intersectionDecision == 0: #left
                self.trajectory = self.left_trajectory
            elif self.intersectionDecision == 1: #straight
                self.trajectory = self.straight_trajectory
            else:
                self.trajectory = self.right_trajectory
        if self.initialPoints is None:
            self.set_current_angle()
            print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            # print("begin adjusting angle...")
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.odomTimer = rospy.Time.now()
            self.intersectionState = 1 if self.intersectionDecision!=2 else 1#adjusting angle:0, trajectory following:1, adjusting angle2: 2..
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        print("position: ",x,y)
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if self.yaw>=5.73: #subtract 2pi to get error between -pi and pi
                error-=6.28
            # print("yaw, curAngle, error: ", self.yaw, self.currentAngle, error)
            if abs(error) <= 0.05:
                self.intersectionState+=1 #done adjusting
                print("done adjusting angle. Transitioning to trajectory following")
                print(f"current position: ({self.odomX},{self.odomY})")
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed*0.7)
                return 0
        elif self.intersectionState==1: #trajectory following
            desiredY = self.trajectory(x)
            error = y - desiredY
            # print("x, y_error: ",x,abs(error) )
            # if x>=(self.offsets_x[self.intersectionDecision]-0.1) and (abs(error)<=0.35):
            # arrived = (x>=(self.offsets_x[self.intersectionDecision]) and abs(y)>=self.offsets_y[self.intersectionDecision]) or abs(self.yaw-self.destinationAngle)<= 0.32
            arrived = abs(self.yaw-self.destinationAngle) <= 0.15
            # print("yaw_error: ")
            # print(str(self.yaw-self.destinationAngle))
            if arrived:
                # print("trajectory done. adjust angle round 2")s
                self.intersectionState += 1
                self.last_error2 = 0 #reset pid errors
                self.error_sum2 = 0
                return 0
            steering_angle = self.pid2(error)
            print("steering: ",steering_angle)
            # print("x, y, desiredY, angle, steer: ", x, y, desiredY, self.yaw, steering_angle*180/3.14159)
            self.publish_cmd_vel(steering_angle, self.maxspeed*0.75)
            return 0
        elif self.intersectionState == 2: #adjust angle 2
            error = self.yaw-self.destinationAngle
            if self.yaw>=5.73: #subtract 2pi to get small error
                error-=6.28
            # print("yaw, destAngle, error: ", self.yaw, self.destinationAngle, error)
            if abs(error) <= 0.15:
                print("done adjusting angle!!")
                self.doneManeuvering = True
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                steering_angle = self.pid(error)
                self.publish_cmd_vel(steering_angle, self.maxspeed*0.7)
                return 0
    def maneuverIntHC(self):
        if self.doneManeuvering:
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            self.state = 0 #go back to lane following
            return 1
        elif self.intersectionDecision <0: 
            self.intersectionDecision = 2 #replace this with service call
            # self.intersectionDecision = np.random.randint(low=0, high=3) #replace this with service call
            print("intersection decision: going " + self.intersectionDecisions[self.intersectionDecision])
        if self.intersectionDecision == 0: #left
            #go straight for 2.5s then left for 4.5s
            if self.timer is None and self.timer2 is None: #begin going straight
                print("begin going straight")
                self.timer = rospy.Time.now()+rospy.Duration(2.5)
            if self.timer is not None and self.timer2 is None:
                if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                    print("finished going straight. reset timer to None")
                    self.timer = None
                    self.timer2 = rospy.Time.now()+rospy.Duration(4.5)
                else:
                    self.straight(0.2)
                    return 0
            if self.timer is None and self.timer2 is not None: #begin going left
                if rospy.Time.now() >= self.timer2: #finished going left
                    print("finished going left. reset timer2 to None. Maneuvering done")
                    self.timer2 = None #finished going left. reset timer2 to None.
                    self.doneManeuvering = True
                    return 0
                else: 
                    self.left(0.12)
                    return 0 
        elif self.intersectionDecision == 1: #straight
            #go straight for 3.7s
            if self.timer is None: #begin going straight
                print("begin going straight")
                self.timer = rospy.Time.now()+rospy.Duration(3.7)
            if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                print("finished going straight. reset timer to None")
                self.timer = None
                self.doneManeuvering = True
                return 0
            else:
                self.straight(0.2)
                return 0
        elif self.intersectionDecision == 2: #right
            #go straight for 0.5s then right for 3s
            if self.timer is None and self.timer2 is None: #begin going straight
                print("begin going straight")
                self.timer = rospy.Time.now()+rospy.Duration(0.1)
            if self.timer is not None and self.timer2 is None:
                if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                    print("finished going straight. reset timer to None")
                    self.timer = None
                    self.timer2 = rospy.Time.now()+rospy.Duration(6.5)
                else:
                    self.straight(self.maxspeed)
                    return 0
            if self.timer is None and self.timer2 is not None: #begin going left
                if rospy.Time.now() >= self.timer2: #finished going straight
                    print("finished going right. reset timer2 to None. Maneuvering done")
                    self.timer2 = None #finished going left. reset timer2 to None.
                    self.doneManeuvering = True
                    return 0
                else: 
                    self.right(0.12)
                    return 0
        return 0       
    def approachCrosswalk(self):
        #Transition events
        if self.timer is None: #start timer. ~13 seconds to pass crosswalk
            print("slowing down to "+str(0.66*self.maxspeed)+"m/s")
            self.timer = rospy.Time.now() + rospy.Duration(13)
        if rospy.Time.now() >= self.timer:
            print("crosswalk passed, speed back up to "+str(self.maxspeed)+"m/s")
            self.timer = None #reset timer
            self.state = 0
            return 1
        elif self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.timer = None #reset timer
            self.pedestrian_sem = 20 #set semaphore for pedestrian
            self.history = self.state
            self.state = 5
            return 1
        #Action: slow down
        # Publish the steering command
        self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed*0.66) #Slower
        return 0
    def stopPedestrian(self):
        if self.pedestrian_clears():
            self.pedestrian_sem-=1
            if self.pedestrian_sem<=0:
                self.state = self.history if self.history is not None else 0
                self.history = None
                return 1
        else:
            self.pedestrian_sem=20
        #Action: idle
        self.idle()
        return 0 
    def highway(self):
        if self.highway_exit_detected():
            if self.entering_roundabout():
                self.state = 8
            else:
                self.state = 0
        self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed*1.33) 
    def carBlock(self):
        #/entry: checkDotted
        #action: overtake or wait
        if self.carCleared is None:
            self.carCleared = False
        if self.dotted:#use service
            #overtake
            if self.timer is None and self.timer2 is None and self.timer3 is None: #begin going left
                print("begin going left")
                self.timer = rospy.Time.now()+rospy.Duration(2.0)
            if self.timer is not None and self.timer2 is None and self.timer3 is None:
                if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                    print("finished going left. reset timer to None")
                    self.timer = None
                    self.timer2 = rospy.Time.now()+rospy.Duration(3.0)
                else:
                    self.left(0.12)
                    return 0
            if self.timer is None and self.timer2 is not None and self.timer3 is None: #begin going straight
                if rospy.Time.now() >= self.timer2: #finished going straight
                    print("finished going straight. reset timer2 to None. back to lane")
                    self.timer2 = None #finished going left. reset timer2 to None.
                    self.timer3 = rospy.Time.now()+rospy.Duration(2.0)
                    return 0
                else: 
                    self.straight(0.2)
                    return 0 
            if self.timer is None and self.timer2 is None and self.timer3 is not None: #go back to lane
                if rospy.Time.now() >= self.timer3: #finished going straight
                    print("done overtaking. back to lane following")
                    self.timer3 = None #finished going left. reset timer2 to None.
                    self.carCleared = True
                    return 0
                else: 
                    self.left(0.12)
                    return 0 
        else: #wait
            if self.object_detected(12):
                self.idle()
            else:
                self.carCleared = True
            return 0
    def park(self):
        if self.doneParking:
            print("done parking maneuvering. Stopping vehicle...")
            self.doneParking = False #reset
            self.state = 11 #parked
            self.initialPoints = None #reset initial points
            return 1
        elif self.parkingDecision <0:
            self.parkingDecision = self.parkDec[self.parkDecI] #replace this with service call
            self.parkDecI+=1
            print("parking decision: going " + self.parkingDecisions[self.parkingDecision])
            if self.parkingDecision == 0: #leftParking
                self.trajectory = self.leftpark_trajectory
            elif self.parkingDecision == 1: #noParking
                self.doneParking = False #reset
                self.state = 0 #lane Following
                self.initialPoints = None #reset initial points
                return 1
            elif self.parkingDecision == 2: #rightParking
                self.trajectory = self.right_trajectory
        if self.parkingDecision == 3:
            if self.initialPoints is None:
                self.set_current_angle()
                print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                print("begin going straight for 0.75m...")
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.odomTimer = rospy.Time.now()
                self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
            self.odometry()
            poses = np.array([self.odomX, self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x, y = poses[0], poses[1]
            print("position: ",x,y)
            if self.intersectionState==0: #going straight
                if x >= 0.75:
                    self.intersectionState = 1
                    print("done going straight. begin adjusting angle...")
                    print("current angle, destination: ", self.yaw, self.destinationAngle)
                self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed*0.75)
                return 0
            if self.intersectionState==1: #adjusting
                error = self.yaw - self.destinationAngle
                if self.yaw>=5.73: #subtract 2pi to get small error
                    error-=6.28
                # print("yaw, error: ", self.yaw, error)
                if abs(error) >= 35*np.pi/180:
                    self.intersectionState = 3 # skip adjusting 2
                    print("35 degrees...")
                    # print("going back for 0.3m...")
                    self.timer5 = rospy.Time.now()+rospy.Duration(3) #change to odom
                self.publish_cmd_vel(23, -self.maxspeed*0.75)
                return 0
            elif self.intersectionState==2: #adjusting
                if rospy.Time.now() >= self.timer5:
                    self.intersectionState = 3
                    self.timer5 = None
                    print("done going back. begin adjusting angle round2...")
                self.straight(-self.maxspeed*0.75)
                return 0
            elif self.intersectionState==3: #adjusting
                error = self.yaw - self.destinationAngle
                if self.yaw>=5.73: #subtract 2pi to get small error
                    error-=6.28
                if abs(error) < 0.05:
                    print("done")
                    self.doneParking = True
                    return 0
                self.publish_cmd_vel(-23, -self.maxspeed*0.75)
                return 0
        elif self.parkingDecision == 2:
            if self.initialPoints is None:
                self.set_current_angle()
                print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                print("initialPoints points: ", self.initialPoints)
                self.offset = 0.1
                print("begin going straight for "+str(self.offset)+"m")
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.odomTimer = rospy.Time.now()
                self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
            self.odometry()
            poses = np.array([self.odomX, self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x, y = poses[0], poses[1]
            print("position: ",x,y)
            if self.intersectionState==0: #adjusting
                if abs(x)>=self.offset:
                    self.intersectionState+=1 #done adjusting
                    self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                    self.odomTimer = rospy.Time.now()
                    print("done going straight. Transitioning to trajectory following")
                    print(f"current odom position: ({self.odomX},{self.odomY})")
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                else:
                    self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed)
                    print(str(x))
                    return 0
            elif self.intersectionState==1: #trajectory following
                # poses = np.array([self.odomX,self.odomY])
                # poses = poses.dot(self.rotation_matrices[self.orientation])
                # print("subtracted by offsets: ", poses)
                # print("after transformation: ", poses)
                # x,y = poses[0], poses[1]
                desiredY = self.trajectory(x)
                error = y - desiredY
                # print("x, y error: ",x,abs(error) )
                if x>=(self.offsets_x[self.intersectionDecision]-0.1) and abs(y)>=(self.offsets_y[self.intersectionDecision]-0.2):# might need to change
                    print("trajectory done. adjust angle round 2")
                    self.intersectionState += 1
                    self.last_error2 = 0 #reset pid errors
                    self.error_sum2 = 0
                    return 0
                steering_angle = self.pid2(error)
                # print("x, y, desiredY, angle: ", x, y, desiredY, steering_angle*57.29578)
                self.publish_cmd_vel(steering_angle, self.maxspeed*0.75)
                return 0
            elif self.intersectionState == 2: #adjust angle 2
                error = self.yaw-self.destinationAngle
                if self.yaw>=5.73: #subtract 2pi to get small error
                    error-=6.28
                # print("yaw, destAngle, error: ", self.yaw, self.destinationAngle, error)
                if abs(error) <= 0.05:
                    print("done adjusting angle!!")
                    print("adjusting position to y between 0.3-0.4")
                    self.intersectionState += 1
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                else:
                    if abs(y)<0.3: #adjust forward
                        self.publish_cmd_vel(self.pid(error), self.maxspeed*0.6)
                        self.parkAdjust = True
                    elif abs(y)>0.4: #adjust backward
                        self.publish_cmd_vel(-self.pid(error), -self.maxspeed*0.6)
                        self.parkAdjust = False
                    elif self.parkAdjust:
                        self.publish_cmd_vel(self.pid(error), self.maxspeed*0.6)
                    else:
                        self.publish_cmd_vel(-self.pid(error), -self.maxspeed*0.6)
                    return 0
            elif self.intersectionState == 3: #adjust position
                if abs(y)<0.3:
                    self.straight(self.maxspeed*0.6)
                    return 0
                elif abs(y)>0.4:
                    self.straight(-self.maxspeed*0.6)
                    return 0
                else:
                    print("done adjusting position.")
                    print(f"current odom position: ({self.odomX},{self.odomY})")
                    self.doneParking = True
                    return 0

    #transition events
    def can_park(self):#not implemented yet
        return False
    def entering_roundabout(self):
        return self.object_detected(3)
    def stop_sign_detected(self):
        return self.object_detected(2)
    def highway_entrance_detected(self):
        return self.object_detected(7)
    def highway_exit_detected(self):
        return self.object_detected(1)
    def light_detected(self):
        return self.object_detected(9)
    def parking_detected(self):
        return self.object_detected(4)
    def is_green(self): #not implemented yet
        return False #call service or message
    def crosswalk_sign_detected(self):
        return self.object_detected(5)
    def pedestrian_appears(self):# change that
        return self.object_detected(7) or self.object_detected(1) or self.object_detected(11)
    def pedestrian_clears(self):# change that
        return not (self.object_detected(7) or self.object_detected(1) or self.object_detected(11))

    #controller functions
    def straight(self,speed):
        # self.cmd_vel_pub(0.0, speed)
        # print("straight at: "+str(speed))
        self.msg.data = '{"action":"1","speed":'+str(speed)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)
    def left(self,speed):
        # self.cmd_vel_pub(-23, speed)
        # print("left at: "+str(speed))
        self.msg.data = '{"action":"1","speed":'+str(speed)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(-23.0)+'}'
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)
    def right(self,speed):
        # self.cmd_vel_pub(23, speed)
        # print("right at: "+str(speed))
        self.msg.data = '{"action":"1","speed":'+str(speed)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(23.0)+'}'
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)
    def idle(self):
        # self.cmd_vel_pub(0.0, 0.0)
        self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        self.cmd_vel_pub.publish(self.msg)

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
    def left_trajectory(self, x):
        return math.exp(3.57*x-4.3)
    def straight_trajectory(self, x):
        return 0
    def right_trajectory(self, x):
        return -math.exp(4*x-3)
        # return -math.exp(3.75*x-3.33)
    def leftpark_trajectory(self, x):
        return math.exp(3.57*x-4.2) #real dimensions
    def set_current_angle(self):
        self.orientation = np.argmin([abs(self.yaw),abs(self.yaw-1.5708),abs((self.yaw)-3.14159),abs(self.yaw-4.71239),abs(self.yaw-6.28319)])%4
        self.currentAngle = self.orientations[self.orientation]
        if self.intersectionDecision == 0 or self.parkingDecision == 0: #left
            self.destinationOrientation = self.directions[(self.orientation+1)%4]
            self.destinationAngle = self.orientations[(self.orientation+1)%4]
            return
        elif self.intersectionDecision == 1: #straight
            self.destinationOrientation = self.orientation
            self.destinationAngle = self.currentAngle
            return
        elif self.intersectionDecision == 2 or self.parkingDecision == 2: #right
            self.destinationOrientation = self.directions[(self.orientation-1)%4]
            self.destinationAngle = self.orientations[(self.orientation-1)%4]
            return
        elif self.parkingDecision ==3:
            self.destinationOrientation = self.directions[(self.orientation)%4]
            self.destinationAngle = self.orientations[(self.orientation)%4]
            return
    def object_detected(self, obj_id):
        if self.numObj >= 2:
            if self.detected_objects[0]==obj_id: 
                if self.check_size(obj_id,0):
                    return True
            elif self.detected_objects[1]==obj_id:
                if self.check_size(obj_id,1):
                    return True
        elif self.numObj == 1:
            if self.detected_objects[0]==obj_id: 
                if self.check_size(obj_id,0):
                    return True
        return False
    def check_size(self, obj_id, index):
        #checks whether a detected object is within a certain min and max sizes defined by the obj type
        box = self.box1 if index==0 else self.box2
        size = max(box[2], box[3])
        return size >= self.min_sizes[obj_id] and size <= self.max_sizes[obj_id]
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
            steering_angle = np.clip(steering_angle, -0.4, 0.4)
        self.msg.data = '{"action":"1","speed":'+str(velocity)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(steering_angle*180/np.pi)+'}'
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)

if __name__ == '__main__':
    node = StateMachine()
    while not rospy.is_shutdown():
        node.rate.sleep()
        rospy.spin()