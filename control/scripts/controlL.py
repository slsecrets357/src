#!/usr/bin/env python3

import rospy
import json
import cv2
import os
import time
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
#from pynput import keyboard
from std_msgs.msg import String

class LaneFollower():
    def __init__(self):

        # loading hough line parameters
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'r')
        data = json.load(file)
        print(data)
        self.pida = False
        self.point = np.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = np.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.p = 0.006
        self.stopline = False
        self.inter_dec = 'straight'
        self.maxspeed = 0.1
        self.i = 0
        self.d = 0.003
        self.last = 0
        self.center = 0
        self.t1 = None
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.cd = rospy.Time.now()
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        
        # self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)

        self.rate = rospy.Rate(10)
        # keyboard listener
        # self.keyInput()

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        

            # msg.data = '{"action":"4","activate":'+'true'+'}'
            # node.cmd_vel_pub.publish(msg)
        # msg.data = '{"action":"1","speed":'+str(0.1)+'}'
        # node.cmd_vel_pub.publish(msg)
        # Convert the image to the OpenCV format
        image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        # cv2.imshow('1', image)
        # cv2.waitKey(1)
        # stopline decision
        # if self.stopline and rospy.Time.now()>=self.cd:
        #     print('decision')
        #     self.intersection_decision()
        #     self.stopline = False
        #     self.cd = rospy.Time.now()+rospy.Duration(1.5)
        # if self.t1 is None:
        #     self.t1 = rospy.Time.now() + rospy.Duration(4)
        # if rospy.Time.now() <= self.t1:
        #     msg.data = '{"action":"4","activate":'+'true'+'}'
        #     node.cmd_vel_pub.publish(msg)
        # else:

        if self.stopline:
            self.stop(10)

            # Extract the lane center from the image
        center = self.histogram(image)

        # Determine the steering angle based on the center
        steering_angle = self.get_steering_angle(center)

        # Publish the steering command
        self.publish_cmd_vel(steering_angle)
        # msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'

        # self.cmd_vel_pub.publish(msg)

    def intersection_decision(self):
        """
        Decision process
        """
        self.stop(0.4)
        
        rospy.loginfo("intersection detected, current decision: %s", self.inter_dec)
        if self.inter_dec=='left':
            rospy.loginfo('turn left')
            self.go_straight(3.5)
            self.turn_left(8)
        elif self.inter_dec=='right':
            rospy.loginfo('turn right')
            self.go_straight(1.5)
            self.turn_right(8)
        elif self.inter_dec=='straight':
            rospy.loginfo('go straight')
            self.go_straight(0.3)
        elif self.inter_dec=='idle':
            rospy.loginfo('idle')
            rospy.loginfo('destination reached')
            self.idle()

    def stop(self,t):
        """
        Stop the car for t seconds
        :param t: stop time
        """
        rospy.loginfo("stop function called")
        msg = String()
        msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
        self.cmd_vel_pub.publish(msg)
        end_time = rospy.Time.now()+rospy.Duration(t)
        while rospy.Time.now()<end_time:
            self.rate.sleep()

    def go_straight(self,t):
        """
        Go straight
        :param t: travel time
        """
        msg = String()
        if not self.pida:
            self.pida = True
            msg.data = '{"action":"4","activate":'+'true'+'}'
            t_end = rospy.Time.now()+rospy.Duration(10)
            while rospy.Time.now()<t_end:
                self.cmd_vel_pub.publish(msg)
                self.rate.sleep()
        rospy.loginfo("go straight called %.2f seconds", t)
        # msg = String()
        msg.data = '{"action":"1","speed":'+str(self.maxspeed)+'}'
        t_end = rospy.Time.now()+rospy.Duration(3)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        # msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        # self.cmd_vel_pub.publish(msg)

    def go_back(self,t):
        """
        Go straight
        :param t: travel time
        """
        # rospy.loginfo("go back called %.2f seconds", t)
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(-0.2)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(0)+'}'
        t_end = rospy.Time.now()+rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg2)
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()
        msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        self.cmd_vel_pub.publish(msg)

    def idle(self):
        """
        Stop the car till not idle
        :param t: stop time
        """
        msg = String()
        while True:
            msg.data = '{"action":"3","steerAngle":'+str(0)+'}'
            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

    def turn_left(self,t):
        """
        Turn left
        :param t: turn time
        """
        self.intersection_detected = True
        self.inter_dec = 'left'
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.12)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(-23)+'}'
        t_end = rospy.Time.now() + rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub.publish(msg2)
            self.rate.sleep()

    def turn_right(self,t):
        """
        Turn right
        :param t: turn time
        """
        self.inter_dec = 'right'
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.12)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(23)+'}'
        t_end = rospy.Time.now() + rospy.Duration(t)
        while rospy.Time.now()<t_end:
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_pub.publish(msg2)

    def houghlines(self, image):
        """
        Extract the lanes from the image
        :param image: Image data in the OpenCV format
        :return: Tuple containing the left and right lanes (each a list of points)
        """
        # Convert the image to grayscale and apply a blur to remove high frequency noise
        # Apply a Canny edge detector to find the edges in the image
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),(5,5),0),50,150)

        # Create a mask for the region of interest (ROI) in the image where the lanes are likely to be
        mask = np.zeros_like(edges)
        h = image.shape[0]
        w = image.shape[1]
        vertices = np.array([[(0,h*0.8),(self.point[0]*w,self.point[1]*h),(w,0.8*h),(w,h),(0,h)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges,self.res,np.pi/180,self.threshold,minLineLength=self.minlength)

        # Separate the lines into left and right lanes
        left=[]
        right=[]
        self.stopline = False
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line.reshape(4)
                if (x2-x1)==0 or abs((y2-y1)/(x2-x1)) > 0.1:
                    m = (x2-x1)/(y2-y1)
                    p_y = int(self.error_p[1]*h)
                    p_x = int(x1 + (p_y-y1)*m)
                    if p_x < w/2:
                        if p_x < int((self.error_p[0]+self.error_w)*w) and p_x > int(self.error_p[0]*w):
                            left.append(p_x)
                    else:
                        if p_x < int((1-self.error_p[0])*w) and p_x > int((1-self.error_p[0]-self.error_w)*w):
                            right.append(p_x)
                else:
                    if abs(x1-x2)>w/10 and y1>0.85*h:
                        self.stopline = True
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = np.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = np.mean(right)
        return (left_lane+right_lane)/2

    def histogram(self, image):
        self.stopline = False
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = np.zeros_like(img_gray)
        # poly = np.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = np.array([[(int(0*w),int(0.85*h)),(int(1*w),int(0.85*h)),(w,h),(0,h)]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 100, 255, cv2.THRESH_BINARY)
        hist=np.zeros((1,w))
        for i in range(w):
            hist[0,i]=np.sum(thresh[:,i])
        lanes=[]
        p=0
        for i in range(w):
            if hist[0,i]==255 and p==0:
                lanes.append(i)
                p=255
            elif hist[0,i]==0 and p==255:
                lanes.append(i)
                p=0
        if len(lanes)%2==1:
            lanes.append(w-1)
        centers=[]
        for i in range(int(len(lanes)/2)):
            if abs(lanes[2*i]-lanes[2*i+1])>350:
                self.stopline = True
            elif abs(lanes[2*i]-lanes[2*i+1])>3:
                centers.append((lanes[2*i]+lanes[2*i+1])/2)
        if len(centers)==0:
            # no lane detected
            self.leftlane = -1
            self.rightlane = w
            return w/2
        elif len(centers)==1:
            # one lane detected
            if centers[0]>w/2:
                # rightlane
                self.leftlane = -1
                self.rightlane = centers[0]
                return (centers[0]+0)/2
            else:
                # leftlane
                self.leftlane = centers[0]
                self.rightlane = w
                return (centers[0]+w)/2
        elif abs(centers[len(centers)-1]-centers[0])<200:
            # group lanes together
            if (centers[len(centers)-1]+centers[0])>w:
                self.leftlane = -1
                self.rightlane = (centers[len(centers)-1]+centers[0])/2
                return ((centers[len(centers)-1]+centers[0])/2+0)/2
            else:
                self.leftlane = (centers[len(centers)-1]+centers[0])/2
                self.rightlane = w
                return ((centers[len(centers)-1]+centers[0])/2+w)/2
        else:
            # rightlane - leftlane
            self.leftlane = centers[0]
            self.rightlane = centers[len(centers)-1]
            return (centers[len(centers)-1]+centers[0])/2

    def dotted_lines(self,image):
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = np.zeros_like(img_gray)
        # poly = np.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = np.array([[(0,int(0.5*h)),(0,h),(int(0.4*w),h),(int(0.4*w),int(0.5*h))]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY)
        hist=np.zeros((int(0.5*h),1))
        v=int(0.5*h)
        for i in range(int(0.5*h)):
            hist[i,0]=np.sum(thresh[i+v,:])
        t = np.mean(hist)
        lanes=[]
        p=0
        for i in range(int(0.5*h)):
            if hist[i,0]>t and p==0:
                lanes.append(i)
                p=t
            elif hist[i,0]<t/2 and p==t:
                lanes.append(i)
                p=0
        if len(lanes)%2==1:
            lanes.append(int(0.5*h)-1)
        if len(lanes)>5:
            return True
        else:
            return False

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

    def publish_cmd_vel(self, steering_angle):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        msg = String()
        msg2 = String()
        x = self.maxspeed + self.maxspeed*abs(steering_angle)/0.4
        msg.data = '{"action":"1","speed":'+str(x)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(steering_angle*180/np.pi)+'}'
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub.publish(msg2)
        # rospy.loginfo("published data: %s", msg.data)

#    def keyInput(self):
#        self.allKeys = ['=','-','w','s','r','l','g','p','e','1']
#        with keyboard.Listener(on_press = self.keyPress) as listener:
#            listener.join()

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
                    msg.data = '{"action":"1","speed":'+str(self.maxspeed)+'}'
                    self.cmd_vel_pub.publish(msg)
                elif key.char == 'e':
                    self.maxspeed -= 0.01
                    msg.data = '{"action":"1","speed":'+str(self.maxspeed)+'}'
                    self.cmd_vel_pub.publish(msg)
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
                elif key.char == '1':
                    self.maxspeed = 0
        except: pass

if __name__ == '__main__':
    try:
        node = LaneFollower()
        # node.stop(3)
        msg = String()
        # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        # node.cmd_vel_pub.publish(msg)
        # t_end = rospy.Time.now() + rospy.Duration(5)
        # print("init")
        # while not rospy.is_shutdown:
        #     t1 = rospy.Time.now() + rospy.Duration(5)
        #     if rospy.Time.now() <= t1:
        #         msg.data = '{"action":"4","activate":'+'true'+'}'
        #         node.cmd_vel_pub.publish(msg)

        #     # msg.data = '{"action":"4","activate":'+'true'+'}'
        #     # node.cmd_vel_pub.publish(msg)
        #     msg.data = '{"action":"1","speed":'+str(0.1)+'}'
        #     node.cmd_vel_pub.publish(msg)
        # print("PIDA")

        # node.go_straight(3)
        # while rospy.Time.now()<t_end:
        #     msg = String()
        #     msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        #     node.cmd_vel_pub.publish(msg)
        # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        # node.cmd_vel_pub.publish(msg)

        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        node.cmd_vel_pub.publish(msg)
        cv2.destroyAllWindows()

