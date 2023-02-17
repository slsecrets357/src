#!/usr/bin/env python3
import rospy
import numpy as np
from message_filters import ApproximateTimeSynchronizer
from std_msgs.msg import String, Byte
from utils.msg import Lane, Sign
from pynput import keyboard
from utils.srv import *
import message_filters
class LaneFollower():
    def __init__(self):
        #states
        self.states = ['Lane Following', "Approaching Intersection", "Stopping at Intersection", "Intersection Maneuvering", "Approaching Crosswalk", "Pedestrian"]
        self.state = 0

        #sign
        self.class_names = ['oneway', 'highwayexit', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayentrance', 'priority',
                'lights','block','pedestrian','car','others','nothing']
        self.min_sizes = [50,50,55,50,50,65,50,50,60,25,50,100,100]
        self.max_sizes = [50,50,55,50,50,65,50,50,60,25,50,100,100]
        self.center = -1
        self.detected_objects = []
        self.numObj = -1
        self.box1 = []
        self.box2 = []

        #steering
        self.p = 0.006
        self.ArrivedAtStopline = False
        self.maxspeed = 0.15
        self.i = 0
        self.d = 0.003
        self.last = 0
        self.center = 0

        #other
        self.timer = None
        self.timer2 = None
        self.intersectionStop = None
        self.intersectionDecision = -1 #0:left, 1:straight, 2:right
        self.intersectionDecisions = ["left", "straight", "right"] #0:left, 1:straight, 2:right
        self.doneManeuvering = False
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.cd = rospy.Time.now()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.rate = rospy.Rate(10)
        # Subscribe to topics
        self.lane_sub = message_filters.Subscriber('lane', Lane, queue_size=3)
        self.sign_sub = message_filters.Subscriber('sign', Sign, queue_size=3)

        # Create an instance of TimeSynchronizer
        ts = ApproximateTimeSynchronizer([self.lane_sub, self.sign_sub], queue_size=3, slop=0.15)
        ts.registerCallback(self.callback)

        # Create service proxy
        self.get_dir = rospy.ServiceProxy('get_direction',get_direction)
        rospy.wait_for_service('get_direction')
    
    #callback function
    def callback(self,lane,sign):
        # Perform decision making tasks
        # Compute the steering angle & linear velocity
        # Publish the steering angle & linear velocity to the /automobile/command topic
        self.center = lane.center
        self.detected_objects = sign.objects
        self.numObj = sign.num
        self.box1 = sign.box1
        self.box2 = sign.box2
        self.ArrivedAtStopline = lane.stopline
        for i in range(self.numObj):
            if i == 0:
                print(f"{self.class_names[self.objects[i]]} detected! width, height: {self.box1[2]}, {self.box1[3]}")
            elif i == 1:
                print(f"{self.class_names[self.objects[i]]} detected! width, height: {self.box2[2]}, {self.box2[3]}")
            else:
                print(f"{self.class_names[self.objects[i]]} detected!")
        act = self.action()
        if act==1:
            print(f"transitioning to '{self.states[self.state]}'")
        # steering_angle = self.get_steering_angle(self.center)
        # print("steering angle: ", steering_angle)
        # # Publish the steering command
        # self.publish_cmd_vel(steering_angle) 

    #state machine
    def action(self):
        if self.state==0: #lane following
            # Determine the steering angle based on the center
            steering_angle = self.get_steering_angle(self.center)
            print("steering angle: ", steering_angle)
            # Publish the steering command
            self.publish_cmd_vel(steering_angle) 
            #transition guards
            if self.stop_sign_detected():
                self.intersectionStop = True
                self.state = 1
                return 1
            elif self.light_detected():
                #call service to check light color
                if self.is_green():
                    self.intersectionStop = True
                else:
                    self.intersectionStop = False
                self.state = 1
                return 1
            elif self.crosswalk_sign_detected():
                self.state = 4
                return 1
            elif self.pedestrian_appears():
                self.state = 5
                return 1
            return 0
        elif self.state == 1: #Approaching Intersection
            #Transition guards
            if self.arrived_at_intersection():
                if self.intersectionStop:
                    self.state = 2
                    return 1
                else:
                    self.doneManeuvering = False #set to false before entering state 3
                    self.state = 3
                    return 1
            #Action: Adjust Position
            
            return 0
        elif self.state == 2: #Stopping at Intersection
            #Action: idle
            self.idle()
            #Transition Guards
            if self.timer is None:
                self.timer = rospy.Time.now() + rospy.Duration(3.57)
            elif rospy.Time.now() >= self.timer:
                self.timer = None
                self.doneManeuvering = False #set to false before entering state 3
                self.state = 3
                return 1
            return 0
        elif self.state == 3: #Intersection Maneuvering
            #go left, straight or right
            #call service to get direction
            #Transition Guards
            if self.doneManeuvering:
                self.intersectionDecision = -1 #reset
                self.state = 0 #go back to lane following
                return 1
            elif self.intersectionDecision <0: 
                self.intersectionDecision = np.random.randint(low=0, high=3) #replace this with service call
                print("intersection decision: going " + self.intersectionDecisions[self.intersectionDecision])
            if self.intersectionDecision == 0: #left
                #go straight for 3.5s then left for 8s
                if self.timer is None and self.timer2 is None: #begin going straight
                    print("begin going straight")
                    self.timer = rospy.Time.now()+rospy.Duration(3.5)
                if self.timer is not None and self.timer2 is None:
                    if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                        print("finished going straight. reset timer to None")
                        self.timer = None
                        self.timer2 = rospy.Time.now()+rospy.Duration(8.0)
                    else:
                        self.straight()
                        return 0
                if self.timer is None and self.timer2 is not None: #begin going left
                    if rospy.Time.now() >= self.timer2: #finished going straight
                        print("finished going left. reset timer2 to None. Maneuvering done")
                        self.timer2 = None #finished going left. reset timer2 to None.
                        self.doneManeuvering = True
                        return 0
                    else: 
                        self.left()
                        return 0 
            elif self.intersectionDecision == 1: #straight
                #go straight for 3s
                if self.timer is None: #begin going straight
                    print("begin going straight")
                    self.timer = rospy.Time.now()+rospy.Duration(3.0)
                if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                    print("finished going straight. reset timer to None")
                    self.timer = None
                    self.doneManeuvering = True
                    return 0
                else:
                    self.straight()
                    return 0
            elif self.intersectionDecision == 2: #right
                #go straight for 1.5s then right for 8s
                if self.timer is None and self.timer2 is None: #begin going straight
                    print("begin going straight")
                    self.timer = rospy.Time.now()+rospy.Duration(1.5)
                if self.timer is not None and self.timer2 is None:
                    if rospy.Time.now() >= self.timer: #finished going straight. reset timer to None
                        print("finished going straight. reset timer to None")
                        self.timer = None
                        self.timer2 = rospy.Time.now()+rospy.Duration(8.0)
                    else:
                        self.straight()
                        return 0
                if self.timer is None and self.timer2 is not None: #begin going left
                    if rospy.Time.now() >= self.timer2: #finished going straight
                        print("finished going left. reset timer2 to None. Maneuvering done")
                        self.timer2 = None #finished going left. reset timer2 to None.
                        self.doneManeuvering = True
                        return 0
                    else: 
                        self.right()
                        return 0
        elif self.state == 4: #Approaching Crosswalk
            #Transition Guards
            if self.timer is None: #start timer. ~5 seconds to pass crosswalk?
                self.timer = rospy.Time.now() + rospy.Duration(5)
            if rospy.Time.now() >= self.timer:
                self.timer = None #reset timer
                self.state = 0
                return 1
            elif self.pedestrian_appears():
                self.timer = None #reset timer
                self.state = 5
                return 1
            #Action: slow down
            steering_angle = self.get_steering_angle(self.center)
            print("steering angle: ", steering_angle)
            # Publish the steering command
            self.publish_cmd_vel(steering_angle, self.maxspeed/2)
            return 0
        elif self.state == 5: #Pedestrian
            if self.pedestrian_clears():
                self.state = 0
                return 1
            #Action: idle
            self.idle()
            return 0
        else:
             self.state = 0
             return 0

    #Transition events
    def stopline_detected(self):
        return False
    def stop_sign_detected(self):
        return self.object_detected(2)
    def light_detected(self):
        return self.object_detected(9)
    def done_stopping(self):
        #set a timer of 3.57 seconds 
        if self.timer is None:
            self.timer = rospy.Time.now() + rospy.Duration(3.57)
        elif rospy.Time.now() >= self.timer:
            self.timer = None
            return True
        return False
    def arrived_at_intersection(self):
        return self.ArrivedAtStopline
    def is_green(self): #call service or message
        r=self.get_dir(0,0,0,'').dir
        if r.split()[2]=='N' or r.split()[2]=='S':
            topic = 'start'
        else:
            topic = 'master'
        state=rospy.wait_for_message('/automobile/trafficlight/'+topic,Byte)
        if state.data==0:
            print('redlight')
            return False
        elif state.data==1:
            print('yellowlight')
            return False
        else:
            return True
    def crosswalk_sign_detected(self):
        return self.object_detected(5)
    def pedestrian_appears(self):
        return False
    def passed_crosswalk(self):
        return False
    def pedestrian_clears(self):
        return False

    #controller functions
    def straight(self):
        self.cmd_vel_pub(0.0, 0.2)
    def left(self):
        self.cmd_vel_pub(-23, 0.12)
    def right(self):
        self.cmd_vel_pub(23, 0.12)
    def idle(self):
        self.cmd_vel_pub(0.0, 0.0)
    def go_back(self):
        self.cmd_vel_pub(0.0, -0.2)

    #helper functions
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
        size = np.max(box[2], box[3])
        return size >= self.min_sizes[obj_id] and size <= self.max_sizes[obj_id]
    def get_steering_angle(self, center):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in radians
        """
        # Calculate the steering angle
        image_center = 640 / 2

        if center==image_center:
            center=self.center
            self.center=image_center
        else:
            self.center=center
        
        error = (center - image_center)
        d_error = error-self.last
        self.last = error
        steering_angle = (error * self.p+d_error*self.d)
        steering_angle = np.clip(steering_angle, -0.4, 0.4)

        return steering_angle

    def publish_cmd_vel(self, steering_angle, velocity = None):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        msg = String()
        msg2 = String()
        if velocity is None:
            velocity = self.maxspeed + self.maxspeed*abs(steering_angle)/0.4
        msg.data = '{"action":"1","speed":'+str(velocity)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(steering_angle*180/np.pi)+'}'
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub.publish(msg2)

    def keyInput(self):
        self.allKeys = ['=','-','w','s','r','l','g','p','e']
        with keyboard.Listener(on_press = self.keyPress) as listener:
            listener.join()

    # ===================================== KEY PRESS ====================================
    def keyPress(self,key):
        """Processing the key pressing 
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key pressed
        """                                     
        try:
            if key.char in self.allKeys:
                if key.char == '=':
                    self.p += 0.001
                elif key.char == '-':
                    self.p -= 0.001
                elif key.char == 'w':
                    self.maxspeed += 0.01
                elif key.char == 'e':
                    self.maxspeed -= 0.01
                elif key.char == 'r':
                    self.inter_dec = 'right'
                    print(self.inter_dec)
                elif key.char == 'l':
                    self.inter_dec = 'left'
                    print(self.inter_dec)
                elif key.char == 's':
                    self.inter_dec = 'straight'
                    print(self.inter_dec)
                elif key.char == 'i':
                    self.inter_dec = 'idle'
                    print(self.inter_dec)
                elif key.char == 'g':
                    self.inter_dec = ''
                elif key.char == 'p':
                    print('p,maxspeed',self.p,self.maxspeed)
        except: pass

if __name__ == '__main__':
    try:
        node = LaneFollower()
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("interrupt")