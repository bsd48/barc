#!/usr/bin/env python

import rospy
from barc.msg import Encoder
from geometry_msgs.msg import Vector3
from math import pi
# input variables
loop_rate = 5
dt = 1.0/loop_rate
tire_radius = 0.051;
# from encoder
count = 0
temp_encoder = 0
e_0, e_1, e_2 = 0, 0, 0
e_init = 0
received_signal = False
# ecu command update
def encoder_callback(data):
    global temp_encoder, received_signal
    temp_encoder = data.BR
    received_signal = True
def get_dist(e2, e1):
    return 0.25*pi*tire_radius*(e2 - e1)

def update_vel():
    global count
    if count == 0:
        vel = 0
    elif count == 1:
        vel = 0
        global e_init
        e_init = temp_encoder
    elif count == 2:
        global e_1, e_2
        e_1, e_2 = e_2, temp_encoder
        vel = get_dist(e_2, e_init) - get_dist(e_1, e_init)/dt
    else:
        global e_0, e_1, e_2
        e_0, e_1, e_2 = e_1, e_2, temp_encoder
        vel = 0.5*(3*get_dist(e_2, e_init)-4*get_dist(e_1, e_init) + get_dist(e_0, e_init))/dt
    count += 1
    return vel
# state estimation node
def velocity_estimation():
    # initialize node
    rospy.init_node('velocity_estimator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('encoder', Encoder, encoder_callback)
    state_pub = rospy.Publisher('vel_meas', Vector3, queue_size = 10)

    # get external force model
    rate = rospy.Rate(loop_rate)
    while not rospy.is_shutdown():
        if received_signal:
            vel = update_vel()
            print vel
            state_pub.publish(Vector3(vel, 0, 0))
        rate.sleep()

if __name__ == '__main__':
    try:
       velocity_estimation()
    except rospy.ROSInterruptException:
        pass
