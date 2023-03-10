#!/usr/bin/env python3

import sys
sys.path.append('.')
import os.path
import time
import math
import serial

import RPi.GPIO as GPIO

import rospy

from utils.msg import encoder
from std_msgs.msg import Float32, Header

class encoderNODE():
    def __init__(self):
        rospy.init_node('encoderNODE', anonymous=False)

        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate = 19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        # encoder publisher object
        self.rate = rospy.Rate(30)
        self.encoder_publisher = rospy.Publisher("/automobile/encoder", encoder, queue_size=3)

        # def shutdown():
        #     GPIO.cleanup()
        # rospy.on_shutdown(shutdown)
        
    def setupLED(self):
        GPIO.setmode(GPIO.BCM)
        self.fre = 10000
        self.redval = 255
        self.greval = 255
        self.bluval = 255
        self.brival = 255
        GPIO.setup(17,GPIO.OUT) #BLUE
        GPIO.setup(27,GPIO.OUT) #GREEN
        GPIO.setup(22,GPIO.OUT) #RED
        GPIO.setup(16,GPIO.OUT) #VCC
        self.vcc = GPIO.PWM(16,self.fre)
        self.red = GPIO.PWM(22,self.fre)
        self.gre = GPIO.PWM(27,self.fre)
        self.blu = GPIO.PWM(17,self.fre)
        self.vcc.start(self.brival/2.55)
        self.red.start(self.redval/2.55)
        self.blu.start(self.bluval/2.55)
        self.gre.start(self.greval/2.55)
        i=15
        while i > 0:
           i-=1
           self.vcc.ChangeDutyCycle((self.brival)/2.55)
           self.red.ChangeDutyCycle((255-self.redval)/2.55)
           self.blu.ChangeDutyCycle((255-self.bluval)/2.55)
           self.gre.ChangeDutyCycle((255-self.greval)/2.55)

    #================================ RUN ========================================
    def run(self):
        rospy.loginfo("starting encoderNODE")
        # self.setupLED()
        self._getting()

    #================================ GETTING ========================================
    def _getting(self):
        while not rospy.is_shutdown():
            encoderdata = encoder()
            try:
                speed = self.ser.readline() #make sure this is a float32
                s = speed.decode("utf-8")
                colon_index = s.find(':')
                subs = s[colon_index+1:s.find(';', colon_index)]
                f = float(subs)
                # print(f)
                encoderdata.speed  = f
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "bno055"
                encoderdata.header = header
            except:
                pass
            self.encoder_publisher.publish(encoderdata)
            
            # time.sleep(0.1)
        
if __name__ == "__main__":
    encoderNod = encoderNODE()
    encoderNod.run()
