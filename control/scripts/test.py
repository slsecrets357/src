#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('lane_follower_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)
    # msg = String()
    # msg.data = '{"action":"4","True":'+str(0.1)+'}' #activate pid
    # cmd_vel_pub.publish(msg)
    msg = String()
    rate1 = rospy.Rate(10)
    count=0
    while not rospy.is_shutdown():
        if count>=37:
            break
        # message = {}
        # message['action'] = '4'
        # message['activate'] = True
        # print(str(message))
        # msg.data = str(message)
        # msg.data = '{"action":"4","activate":'+str(True)+'}'
        msg.data = """{"action":"4","activate":true}"""
        if count>=15:
            msg.data = '{"action":"1","speed":'+str(0.0)+'}'
            # msg.data = '{"action":"2","steerAngle":'+str(float(0))+'}'
        # msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        # msg.data = "{'action':'4','activate': True}"
        print(msg.data)
        cmd_vel_pub.publish(msg)
        count+=1
        print(count)
        rate1.sleep()    
    count=0
    while not rospy.is_shutdown():
        print('{"action":"2","steerAngle":'+str(float(count%20))+'}')
        msg.data = '{"action":"2","steerAngle":'+str(float(0))+'}'
        # msg.data = '{"action":"1","speed":'+str(0.175)+'}'
        cmd_vel_pub.publish(msg)
        count+=1
        rate1.sleep()


