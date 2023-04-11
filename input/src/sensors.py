import sys
sys.path.append('.')
import os.path
import time
import math
import serial

import rospy
import RPi.GPIO as GPIO

from utils.msg import IMU, encoder, Sensors
from std_msgs.msg import Header

sensorType = "BNO055"

if sensorType == "BNO055":
    import RTIMU
elif sensorType == "MPU6050":
    from mpu6050 import mpu6050

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
