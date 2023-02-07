#!/usr/bin/env python3

# import onnxruntime
# from yolov7 import YOLOv7
import argparse
import rospy
import json
import cv2
import os
import time
import numpy as alex
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# from pynput import keyboard
from std_msgs.msg import String
from std_msgs.msg import Header
from utils.msg import Lane
import scipy
# from message_filters import ApproximateTimeSynchronizer

class LaneDetector():
    def __init__(self, method = 'histogram', show=True):
        self.method = method
        self.show = show
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'r')
        data = json.load(file)
        print(data)
        self.allKeys = ['=','-','w','s','r','l','g','p']
        self.point = alex.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = alex.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.p = 0.006
        self.i = 0
        self.d = 0.003
        self.last = 0
        self.stopline = False
        self.inter_dec = 'stop'
        self.maxspeed = 0.2
        self.inter_dec = ''
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_detector_node', anonymous=True)
        self.pub = rospy.Publisher("lane", Lane, queue_size=2)
        self.p = Lane()
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("automobile/image_raw/compressed", CompressedImage, self.image_callback)
        # self.image_sync = ApproximateTimeSynchronizer([self.image_sub], queue_size = 2, slop=0.1)
        # # Register the image_callback function to be called when a synchronized message is received
        # self.image_sync.registerCallback(self.image_callback)
        self.rate = rospy.Rate(5)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
         # Update the header information
        header = Header()
        header.seq = data.header.seq
        header.stamp = data.header.stamp
        header.frame_id = data.header.frame_id
        # Update the header information in the message
        self.p.header = header

        # Convert the image to the OpenCV format
        image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        # Extract the lanes from the image
        if self.method == 'histogram':
            lanes = self.histogram(image, show=self.show)
        else:
            lanes = self.extract_lanes(image, show=self.show)

        #Determine the steering angle based on the lanes
        self.p.center = lanes

        #determine whether left lane is dotted
        # dotted = self.dotted_lines(image)
        # self.p.dotted = dotted
        
        #determine whether we arrive at intersection
        self.p.stopline = self.stopline
        print(self.p)
        # Publish the steering command
        self.pub.publish(self.p)

    def dotted_lines(self,image):
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = alex.zeros_like(img_gray)
        # poly = alex.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(0,h)]])
        poly = alex.array([[(0,int(0.5*h)),(0,h),(int(0.4*w),h),(int(0.4*w),int(0.5*h))]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY)
        hist=alex.zeros((int(0.5*h),1))
        v=int(0.5*h)
        for i in range(int(0.5*h)):
            hist[i,0]=alex.sum(thresh[i+v,:])
        t = alex.mean(hist)
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

    def histogram(self, image, show=False):
        self.stopline = False
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        mask = alex.zeros_like(img_gray)
        poly = alex.array([[(int(0*w),int(0.85*h)),(int(1*w),int(0.85*h)),(w,h),(0,h)]])
        cv2.fillPoly(mask,poly,255)
        img_roi = cv2.bitwise_and(img_gray,mask)
        ret, thresh = cv2.threshold(img_roi, 150, 255, cv2.THRESH_BINARY)
        hist=alex.zeros((1,w))
        for i in range(w):
            hist[0,i]=alex.sum(thresh[:,i])
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
            center = w/2
        elif len(centers)==1:
            if centers[0]>w/2:
                center = (centers[0]-0)/2
            else:
                center = (centers[0]+600)/2
        elif abs(centers[len(centers)-1]-centers[len(centers)-2])<200:
            if (centers[len(centers)-1]+centers[len(centers)-2])>w:
                center = (centers[len(centers)-1]+centers[len(centers)-2]/2+0)/2
            else:
                center = (centers[len(centers)-1]+centers[len(centers)-2]/2+600)/2
        else:
            center = (centers[len(centers)-1]+centers[len(centers)-2])/2
        if show:
            cv2.line(image,(int(center),int(image.shape[0])),(int(center),int(0.8*image.shape[0])),(255,0,255),5)
            cv2.imshow('center', image)
            cv2.waitKey(1)
        return center

    def extract_lanes(self, image, show=False): #hough transform
        """
        Extract the lanes from the image
        :param image: Image data in the OpenCV format
        :return: Tuple containing the left and right lanes (each a list of points)
        """
        # Convert the image to grayscale and apply a blur to remove high frequency noise
        # Apply a Canny edge detector to find the edges in the image
        t1 = time.time()
        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),(5,5),0),50,150)

        # Create a mask for the region of interest (ROI) in the image where the lanes are likely to be
        mask = alex.zeros_like(edges)
        h = image.shape[0]
        w = image.shape[1]
        vertices = alex.array([[(0,h*0.8),(self.point[0]*w,self.point[1]*h),(w,0.8*h),(w,h),(0,h)]], dtype=alex.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges,self.res,alex.pi/180,self.threshold,minLineLength=self.minlength)

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
                        print(x1,y1,x2,y2)
                        print(len(lines))
                        self.stopline = True
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = alex.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = alex.mean(right)
        print("time used: ", time.time()-t1)
        center = (left_lane+right_lane)/2
        print(center)
        if show:
            cv2.line(image,(int(center),int(image.shape[0])),(int(center),int(0.8*image.shape[0])),(255,0,255),5)
            cv2.imshow('center', image)
            cv2.waitKey(1)
        return (left_lane+right_lane)/2

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=str, default='histogram', help="hough or histogram")
    parser.add_argument("--show", type=str, default=True, help="show camera frames")
    args = parser.parse_args()
    try:
        node = LaneDetector(method=args.method, show = args.show)
        node.rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

