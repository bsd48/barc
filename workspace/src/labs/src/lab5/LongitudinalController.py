#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Vector3
from barc.msg import ECU, Encoder
from numpy import pi

# from encoder
v_meas      = 0.0
t0          = time.time()
r_tire      = 0.05 # radius of the tire
servo_pwm   = 1540.0
motor_pwm   = 1500.0
motor_pwm_offset = 1567

# reference speed 
v_ref = 1.0 # give reference speed is 0.5 m/s

# ===================================PID longitudinal controller================================#
class PID():
    def __init__(self, kp=1, ki=1, kd=0, integrator=0, derivator=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator = integrator
        self.derivator = derivator
        self.integrator_max = 10
        self.integrator_min = -10
        self.control_max = 2000 - motor_pwm_offset
        self.control_min = 1200 - motor_pwm_offset
    def acc_calculate(self, speed_reference, speed_current):
        self.error = speed_reference - speed_current
        
        # Propotional control
        self.P_effect = self.kp*self.error
        
        # Integral control
        self.integrator = self.integrator + self.error
        ## Anti windup
        if self.integrator >= self.integrator_max:
            self.integrator = self.integrator_max
        if self.integrator <= self.integrator_min:
            self.integrator = self.integrator_min
        self.I_effect = self.ki*self.integrator
        
        # Derivative control
        self.derivator = self.error - self.derivator
        self.D_effect = self.kd*self.derivator
        self.derivator = self.error

        acc = self.P_effect + self.I_effect + self.D_effect
        
        # saturation block
        acc = min(max(self.control_min, acc), self.control_max)

        return acc

# =====================================end of the controller====================================#
def vel_est_update(data):
    global v_meas
    v_meas = data.BR
    
# state estimation node
def controller():
    global motor_pwm, servo_pwm, motor_pwm_offset
    global v_ref, v_meas
    
    # Initialize node:
    rospy.init_node('simulationGain', anonymous=True)

    # TODO: Add your necessary topic subscriptions / publications, depending on your preferred method of velocity estimation
    ecu_pub   = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)
    #v_meas_sub = rospy.Subscriber('vel_meas', Vector3, vel_mes_update)
    v_meas_sub = rospy.Subscriber('vel_est', Encoder, vel_est_update)
    # Set node rate
    loop_rate   = 10
    rate        = rospy.Rate(loop_rate)
    
    # TODO: Initialize your PID controller here, with your chosen PI gains
    PID_control = PID(kp = 60, ki = 20, kd = 20)
    
    while not rospy.is_shutdown():
        # calculate acceleration from PID controller.
        motor_pwm = PID_control.acc_calculate(v_ref, v_meas) + motor_pwm_offset
 
        # publish control command
        ecu_pub.publish( ECU(motor_pwm, servo_pwm) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       controller()
    except rospy.ROSInterruptException:
        pass
