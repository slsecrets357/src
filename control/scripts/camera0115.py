#!/usr/bin/env python3

import time
import rospy
import json
import cv2
import os
# from scipy.optimize import curve_fit
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'r')
        data = json.load(file)
        print(data)
        self.point = np.array(data.get('point'))
        self.res = data.get('res')
        self.threshold = data.get('threshold')
        self.minlength = data.get('minlength')
        self.error_p = np.array(data.get('error_p'))
        self.error_w = data.get('error_w')
        self.t = 3
        self.imgL = np.zeros((640, 480))
        self.lane_center = 0
        self.stopline = False
        self.dotline = False
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        windowName = "Frame preview"
        cv2.namedWindow(windowName,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName,960,720)
        # self.trackbars()
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        rospy.init_node('CAMnod', anonymous=True)
        self.rate = rospy.Rate(3)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        # self.image_sub = rospy.Subscriber("/automobile/image_raw/compressed", CompressedImage, self.callback)
        rospy.spin()
    
    def trackbars(self):
        windowName = "Params"
        cv2.namedWindow(windowName,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName,480,360)
        cv2.createTrackbar('Save',windowName,0,1,self.save_object)
        cv2.createTrackbar('View',windowName,0,1,self.view)
        cv2.createTrackbar('point[0]',windowName,int(self.point[0]*100),100,self.changePx)
        cv2.createTrackbar('point[1]',windowName,int(self.point[1]*100),100,self.changePy)
        cv2.createTrackbar('res',windowName,self.res,100,self.changeRes)
        cv2.createTrackbar('threshold',windowName,self.threshold,200,self.changeThreshold)
        cv2.createTrackbar('minlength',windowName,self.minlength,100,self.changeMinlength)
        cv2.createTrackbar('error_p[0]',windowName,int(self.error_p[0]*100),100,self.changePex)
        cv2.createTrackbar('error_p[1]',windowName,int(self.error_p[1]*100),100,self.changePey)
        cv2.createTrackbar('error_w',windowName,int(self.error_w*100),100,self.changeEw)

    def save_object(self,v):
        file = open(os.path.dirname(os.path.realpath(__file__))+'/json-dump.json', 'w')
        data = {"point":self.point.tolist(),"res":self.res,"threshold":self.threshold,"minlength":self.minlength,"error_p":self.error_p.tolist(),"error_w":self.error_w}
        json.dump(data, file)
        self.view(0)

    def view(self,v):
        print("=========== REMOTE CONTROL ============"+'\n'+
        
            "point[0]           "+str(self.point[0])+  '     [L/J]' +
            "\npoint[1]         "+str(self.point[1])+  '     [I/K]' +
            "\nres              "+str(self.res)+       '     [1/2]' +
            "\nthreshold        "+str(self.threshold)+ '     [3/4]' +
            "\nminlength        "+str(self.minlength)+ '     [5/6]' +
            "\nt                "+str(self.t)+         '     [6/7]' +
            "\nerror_p[0]       "+str(self.error_p[0])+'     [h/f]' +
            "\nerror_p[1]       "+str(self.error_p[1])+'     [g/t]' +
            "\nerror_w          "+str(self.error_w)+   '     [e/r]' + 
            "\nlane_center      "+str(self.lane_center/640)+        
            "\nlane_left        "+str(self.left_lane/640)+        
            "\nlane_right       "+str(self.right_lane/640)        
        )

    def changePx(self,v):
        self.point[0] = v/100
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changePy(self,v):
        self.point[1] = v/100
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changeRes(self,v):
        self.res = v
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)

    def changeThreshold(self,v):
        self.threshold = v
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changeMinlength(self,v):
        self.minlength = v
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changePex(self,v):
        self.error_p[0] = v/100
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changePey(self,v):
        self.error_p[1] = v/100
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)
        
    def changeEw(self,v):
        self.error_w = v/100
        self.detect_lines()
        windowName = "Frame preview"
        cv2.imshow(windowName, self.imgL)

    # def poly_func(self,x,a,b,c):
    #     return a*(x**2)+b*x+c

    # def houghlines(self):
    #     img_edge = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY),(5,5),0),50,150)
    #     h = img_edge.shape[0]
    #     w = img_edge.shape[1]
    #     mask = np.zeros_like(img_edge)
    #     poly = np.array([[(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(int(1*w),int(0.8*h)),(w,h),(int(0.65*w),h),(int(0.5*w),int(0.9*h)),(int(0.35*w),h),(0,h)]])
    #     cv2.fillPoly(mask,poly,255)
    #     img_roi = cv2.bitwise_and(img_edge,mask)
    #     lines = cv2.HoughLinesP(img_roi,self.res,np.pi/180,self.threshold,minLineLength=self.minlength)
    #     img_lines = np.copy(self.cv_image)
    #     cv2.line(img_lines,(int(0*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(255,255,255),1)
    #     cv2.line(img_lines,(int(1*w),int(0.8*h)),(int(self.point[0]*w),int(self.point[1]*h)),(255,255,255),1)
    #     cv2.circle(img_lines, (int(self.point[0]*w),int(self.point[1]*h)), radius=3, color=(0,0,0), thickness=-1)
    #     color_e1 = (0,0,255)
    #     color_e2 = (0,0,255)
    #     if lines is not None:
    #         self.lines = len(lines)
    #     else:
    #         self.lines = None
    #     left=[]
    #     right=[]
    #     self.stopline = False
    #     left_x=[]
    #     left_y=[]
    #     right_x=[]
    #     right_y=[]
    #     if lines is not None:
    #         for line in lines:
    #             x1,y1,x2,y2 = line.reshape(4)
    #             if (x2-x1)==0 or abs((y2-y1)/(x2-x1)) > 0.1:
    #                 m = (x2-x1)/(y2-y1)
    #                 p_y = int(self.error_p[1]*h)
    #                 p_x = int(x1 + (p_y-y1)*m)
    #                 if p_x < w/2:
    #                     color = (255,0,0)
    #                     if p_x < int((self.error_p[0]+self.error_w)*w) and p_x > int(self.error_p[0]*w):
    #                         color_e1 = (0,255,0)
    #                         left.append(p_x)
    #                         left_x.append(x1)
    #                         left_y.append(y1)
    #                         left_x.append(x2)
    #                         left_y.append(y2)
    #                         cv2.line(img_lines,(x1,y1),(x2,y2),color,self.t)
    #                         cv2.circle(img_lines, (p_x,p_y), radius=2, color=(0,0,0), thickness=-1)
    #                     else:
    #                         cv2.line(img_lines,(x1,y1),(x2,y2),(100,100,0),self.t)
    #                 else:
    #                     color = (255,255,0)
    #                     if p_x < int((1-self.error_p[0])*w) and p_x > int((1-self.error_p[0]-self.error_w)*w):
    #                         color_e2 = (0,255,0)
    #                         right.append(p_x)
    #                         right_x.append(x1)
    #                         right_y.append(y1)
    #                         right_x.append(x2)
    #                         right_y.append(y2)
    #                         cv2.line(img_lines,(x1,y1),(x2,y2),color,self.t)
    #                         cv2.circle(img_lines, (p_x,p_y), radius=2, color=(0,0,0), thickness=-1)
    #                     else:
    #                         cv2.line(img_lines,(x1,y1),(x2,y2),(100,100,0),self.t)
    #             else:
    #                 if abs(x1-x2)>w/10 and y1>0.85*h:
    #                     cv2.line(img_lines,(x1,y1),(x2,y2),(100,0,100),self.t+2)
    #                     self.stopline = True
    #     cv2.line(img_lines,(int(self.error_p[0]*w),int(self.error_p[1]*h)),(int((self.error_p[0]+self.error_w)*w),int(self.error_p[1]*h)),color_e1,1)
    #     cv2.line(img_lines,(int(self.error_p[0]*w),int((self.error_p[1]-0.05)*h)),(int(self.error_p[0]*w),int((self.error_p[1]+0.05)*h)),color_e1,1)
    #     cv2.line(img_lines,(int((self.error_p[0]+self.error_w)*w),int((self.error_p[1]-0.05)*h)),(int((self.error_p[0]+self.error_w)*w),int((self.error_p[1]+0.05)*h)),color_e1,1)
    #     cv2.line(img_lines,(int((1-self.error_p[0]-self.error_w)*w),int(self.error_p[1]*h)),(int((1-self.error_p[0])*w),int(self.error_p[1]*h)),color_e2,1)
    #     cv2.line(img_lines,(int((1-self.error_p[0]-self.error_w)*w),int((self.error_p[1]-0.05)*h)),(int((1-self.error_p[0]-self.error_w)*w),int((self.error_p[1]+0.05)*h)),color_e2,1)
    #     cv2.line(img_lines,(int((1-self.error_p[0])*w),int((self.error_p[1]-0.05)*h)),(int((1-self.error_p[0])*w),int((self.error_p[1]+0.05)*h)),color_e2,1)
    #     if len(left_x)>3:
    #         popt_l, pcov_l = curve_fit(self.poly_func, left_x, left_y)
    #         x_l = np.arange(np.amin(left_x),np.amax(left_x))
    #         y_l = self.poly_func(x_l,*popt_l)
    #         curve_l = np.column_stack((x_l.astype(np.int32), y_l.astype(np.int32)))
    #         cv2.polylines(img_lines, [curve_l], False, (155,0,0),5)
    #     if len(right_x)>3:
    #         popt_r, pcov_r = curve_fit(self.poly_func, right_x, right_y)
    #         x_r = np.arange(np.amin(right_x),np.amax(right_x))
    #         y_r = self.poly_func(x_r,*popt_r)
    #         curve_r = np.column_stack((x_r.astype(np.int32), y_r.astype(np.int32)))
    #         cv2.polylines(img_lines, [curve_r], False, (155,155,0),5)
    #     self.left_lane = 0
    #     self.right_lane = w
    #     if len(left) == 0:
    #         self.left_lane = 0
    #     else:
    #         self.left_lane = np.mean(left)
    #     if len(right) == 0:
    #         self.right_lane = w
    #     else:
    #         self.right_lane = np.mean(right)
    #     self.lane_center = (self.left_lane+self.right_lane)/2
    #     cv2.line(img_lines,(int(self.lane_center),int(1*h)),(int(self.lane_center),int(0.8*h)),(255,0,255),5)
    #     self.imgL = img_lines

    def histogram(self):
        self.stopline = False
        img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        h = img_gray.shape[0]
        w = img_gray.shape[1]
        img_lines = np.copy(self.cv_image)
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
                print('stopline')
            elif abs(lanes[2*i]-lanes[2*i+1])>3:
                centers.append((lanes[2*i]+lanes[2*i+1])/2)
                cv2.line(img_lines,(int((lanes[2*i]+lanes[2*i+1])/2),int(0.95*h)),(int((lanes[2*i]+lanes[2*i+1])/2),int(0.85*h)),(255,0,0),3)
        if len(centers)==0:
            self.lane_center=w/2
        elif len(centers)==1:
            if centers[0]>w/2:
                self.lane_center = (centers[0]+0)/2
            else:
                self.lane_center = (centers[0]+600)/2
        elif abs(centers[len(centers)-1]-centers[len(centers)-2])<200:
            if (centers[len(centers)-1]+centers[len(centers)-2])>w:
                self.lane_center = (centers[len(centers)-1]+centers[len(centers)-2]/2+0)/2
            else:
                self.lane_center = (centers[len(centers)-1]+centers[len(centers)-2]/2+600)/2
        else:
            self.lane_center = (centers[len(centers)-1]+centers[len(centers)-2])/2
        cv2.line(img_lines,(int(self.lane_center),int(1*h)),(int(self.lane_center),int(0.8*h)),(255,0,255),5)
        self.imgL = img_lines

    def dotted_lines(self):
        self.dotline = False
        img_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
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
            self.dotline = True
            cv2.putText(self.imgL, 'DottedLine!', (int(self.imgL.shape[1]*0.1),int(self.imgL.shape[0]*0.2)), cv2.FONT_HERSHEY_SIMPLEX, 
               1, (0,0,255), 1, cv2.LINE_AA)

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        # self.cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        #self.cv_image+=1
        self.histogram()
        # self.dotted_lines()
        
        cv2.imshow("Frame preview", self.imgL)
        key = cv2.waitKey(1)
            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        pass

