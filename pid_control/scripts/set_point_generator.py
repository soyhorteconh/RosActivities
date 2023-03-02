#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

amplitude = rospy.get_param("/set_point_generator_amplitude", 1.0)

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__== '__main__':
    rospy.init_node('set_point_generator')
    pubSetPoint = rospy.Publisher("/set_point", set_point, queue_size=10)

    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    msg = set_point()
    time = 0
    start_time = rospy.get_time()

    print("The Set Point Genertor is Running")
    while not rospy.is_shutdown():
        time = rospy.get_time() - start_time
        data = amplitude*np.sin(time)
        msg.data = data
        msg.time = time

        rospy.loginfo("Time: %f  Set point: %f ", time, data)
        pubSetPoint.publish(msg)
        rate.sleep()

    