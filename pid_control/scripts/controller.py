#!/usr/bin/env python
import rospy
from pid_control.msg import motor_input
from pid_control.msg import motor_output
from pid_control.msg import set_point
from std_msgs.msg import Float32

import numpy as np

#SetPoint variables
setPoint_data = 0

#Motor Input variables
msg_motor_input = motor_input()

#Motor Output variables
motorOutput_data = 0

#Controller parameters
kp = rospy.get_param("/controller_kp",0.0)
ki = rospy.get_param("/controller_ki",0.0)
kd = rospy.get_param("/controller_kd",0.0)
dt = rospy.get_param("/controller_dt",0.001)

#Controller variables
output = 0
error = 0
errorI = 0
errorD = 0
lastError = 0

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

def setPoint_callback(msg):
    global setPoint_data
    setPoint_data = msg.data

def motorOutput_callback(msg):
    global motorOutput_data
    motorOutput_data = msg.output

def controller(setPoint, motorOutput_data):
    global error, errorI, errorD, output, kp, ki, kd, lastError, dt

    error = setPoint - motorOutput_data
    errorI += error * dt
    errorD = (error - lastError) / dt
    output = kp*error + ki*errorI + kd*errorD

    lastError = error
    msg_motor_input.input = output


if __name__ == '__main__':

    rospy.init_node("controller")
    rospy.Subscriber("/set_point", set_point, setPoint_callback)
    rospy.Subscriber("/motor_output", motor_output, motorOutput_callback)

    kd = rospy.get_param("/controller_kd",0.0)
    pubMotorInput = rospy.Publisher("/motor_input", motor_input, queue_size=10)
    pubError = rospy.Publisher("/error", Float32, queue_size=10)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)    

    while not rospy.is_shutdown():
        controller(setPoint_data, motorOutput_data)
        rospy.loginfo("Controller Time: %f  Controller data: %f", msg_motor_input.time, msg_motor_input.input)
        pubMotorInput.publish(msg_motor_input)
        pubError.publish(error)
        rate.sleep()



