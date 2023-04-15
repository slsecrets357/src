#!/usr/bin/env python3
import sys
sys.path.append('.')
import os.path
import time
import math
import serial
import json

import rospy
import RPi.GPIO as GPIO

from utils.msg import IMU, encoder, Sensors
from std_msgs.msg import Header,String

sensorType = "BNO055"

if sensorType == "BNO055":
    import RTIMU
elif sensorType == "MPU6050":
    from mpu6050 import mpu6050
class MessageConverter:
    commands = {
                '1' : [ ['speed'],               [ float ],                        [False]     ],
                '2' : [ ['steerAngle'],          [ float ],                        [False]     ],
                '3' : [ ['brake (steerAngle)'],  [ float ],                        [False]     ],
                '4' : [ ['activate'],            [ bool  ],                        [False]     ],
                '5' : [ ['activate'],            [ bool  ],                        [False]     ],
                '6' : [ ['kp','ki','kd','tf'],   [ float, float, float, float ],   [True]      ],
                '7' : [ ['distance','speed'],    [ float, float],                  [True]      ]
            }
    """ The 'commands' attribute is a dictionary, which contains key word and the acceptable format for each action type. """   

    # ===================================== GET COMMAND ===================================
    def get_command(self, action, **kwargs):
        """This method generates automatically the command string, which will be sent to the other device. 
        
        Parameters
        ----------
        action : string
            The key word of the action, which defines the type of action. 
        **kwargs : dict
            Optional keyword parameter, which have to contain all parameters of the action. 
            
 
        Returns
        -------
        string
            Command with the decoded action, which can be transmite to embed device via serial communication.
        """
        self.verify_command(action, kwargs)
        
        enhPrec = MessageConverter.commands[action][2][0]
        listKwargs = MessageConverter.commands[action][0]

        command = '#' + action + ':'

        for key in listKwargs:
            value = kwargs.get(key)
            valType = type(value)

            if valType == float:
                if enhPrec:
                    command += '{0:.6f};'.format(value)
                else:
                    command += '{0:.2f};'.format(value)
            elif valType == bool:
                command += '{0:d};'.format(value)   
                         
        command += ';\r\n'
        return command

    # ===================================== VERIFY COMMAND ===============================
    def verify_command(self, action, commandDict):
        """The purpose of this method to verify the command, the command has the right number and named parameters. 
        
        Parameters
        ----------
        action : string
            The key word of the action. 
        commandDict : dict
            The dictionary with the names and values of command parameters, it has to contain all parameters defined in the commands dictionary. 
        """
        
        assert len(commandDict.keys()) == len(MessageConverter.commands[action][0]), \
                'Number of arguments does not match'
        for i, [key, value] in enumerate(commandDict.items()):
            assert key in MessageConverter.commands[action][0], \
                    action + "should not contain key:" + key
            assert type(value) == MessageConverter.commands[action][1][i], \
                    action + "should be of type " + \
                    str(MessageConverter.commands[action][1][i]) + 'instead of' + \
                    str(type(value))
class CombinedNODE():
    def __init__(self):
        # IMU setup
        if sensorType == "BNO055":
            import RTIMU
            self.SETTINGS_FILE = "RTIMULib"
            print("Using settings file " + self.SETTINGS_FILE + ".ini")
            if not os.path.exists(self.SETTINGS_FILE + ".ini"):
                print("Settings file does not exist, will be created")
            self.s = RTIMU.Settings(self.SETTINGS_FILE)
        else:
            from mpu6050 import mpu6050

        # Encoder setup
        self.ser = serial.Serial(
            port='/dev/ttyACM0',
            baudrate = 19200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

        rospy.init_node('combinedNODE', anonymous=False)
        
        self.combined_publisher = rospy.Publisher("/automobile/sensors", Sensors, queue_size=3)

        # serialNODE
        """It forwards the control messages received from socket to the serial handling node. 
        """
        devFile = '/dev/ttyACM0'
        logFile = 'historyFile.txt'
        
        # comm init       
        self.serialCom = serial.Serial(devFile,19200,timeout=0.1)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()

        # log file init
        # self.historyFile = FileHandler(logFile)
        
        # message converted init
        self.messageConverter = MessageConverter()
        # self.buff=""
        # self.isResponse=False
        self.msg = String()
        self.msg.data = '{"action":"5","activate": true}'
        for bhf in range(15):
            # msg.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
            self._write(self.msg)
            time.sleep(0.1)

    
    def _write(self, msg):
        """ Represents the writing activity on the the serial.
        """
        command = json.loads(msg.data)
        # print("hh", type(command), type(msg), type(msg.data))
        # print(msg)
        # print(command)
        # Unpacking the dictionary into action and values
        command_msg = self.messageConverter.get_command(**command)
        self.serialCom.write(command_msg.encode('ascii'))
        # self.historyFile.write(command_msg)

    def run(self):
        rospy.loginfo("starting combinedNODE")
        self._initIMU()
        self._getting()
    
    def _initIMU(self):
        if sensorType == "BNO055":
            self.imu = RTIMU.RTIMU(self.s)
            
            if (not self.imu.IMUInit()):
                sys.exit(1)
            print("IMU Name: " + self.imu.IMUName())
            self.imu.setSlerpPower(0.02)
            self.imu.setGyroEnable(True)
            self.imu.setAccelEnable(True)
            self.imu.setCompassEnable(True)
            self.poll_interval = self.imu.IMUGetPollInterval()
        else:
            self.imu = mpu6050(0x68)
            self.poll_interval = 100

    def _getting(self):
        while not rospy.is_shutdown():
            combined_data = Sensors()
            if self.imu.IMURead():
                combined_data.yaw = math.degrees(self.imu.getIMUData()["fusionPose"][2])
            try:
                speed = self.ser.readline() #make sure this is a float32
                s = speed.decode("utf-8")
                colon_index = s.find(':')
                subs = float(s[colon_index+1:s.find(';', colon_index)])
                combined_data.speed  = subs
            except:
                pass

            self.combined_publisher.publish(combined_data)
            
            time.sleep(self.poll_interval*1.0/1000.0)

if __name__ == "__main__":
    combinedNod = CombinedNODE()
    combinedNod.run()
