#!/usr/bin/env python
import roslib; roslib.load_manifest('rosSimpleStep')
import rospy
from std_msgs.msg import *
from ros_flydra.msg import *
from rosSimpleStep.msg import msgMotorConfig, msgMotorCmd
import time
import numpy as np
import sys
from optparse import OptionParser

sys.path.append("/home/ssafarik/floris")   # So we can find stepper_motor.py

UNITS_STEPS = False
UNITS_RADIANS = True
twopi = 6.283185


class RosSimpleStep:
    
    def __init__(self, idMotor, dummy=False, zerostart=False, units=UNITS_STEPS):
    
        self.idMotor = idMotor
        self.dummy = dummy
        self.STOP_CMD = False
        self.latency = 0.01 
        self.des_vel_filtered = 0.
        self.des_pos_filtered = 0.
        self.zerostart = zerostart
        self.units = units
        self.last = {'pos' : 0, 'vel' : 0}
        
        self.cmds = {
                     # Commands
                    'disable' : self.stepper.disable,
                    'enable' : self.stepper.enable,
                    'get_dir' : self.stepper.get_dir,
                    'get_dir_setpt' : self.stepper.get_dir_setpt,
                    'get_enable' : self.stepper.get_enable,
                    'get_ext_int' : self.stepper.get_ext_int,
                    'get_manufacturer' : self.stepper.get_manufacturer,
                    'get_max_vel' : self.stepper.get_max_vel,
                    'get_min_vel' : self.stepper.get_min_vel,
                    'get_mode' : self.stepper.get_mode,
                    'get_pos' : self.stepper.get_pos,
                    'get_pos_err' : self.stepper.get_pos_err,
                    'get_pos_setpt' : self.stepper.get_pos_setpt,
                    'get_pos_vel' : self.stepper.get_pos_vel,
                    'get_product' : self.stepper.get_product,
                    'get_product_id' : self.stepper.get_product_id,
                    'get_serial_number' : self.stepper.get_serial_number,
                    'get_status' : self.stepper.get_status,
                    'get_vel' : self.stepper.get_vel,
                    'get_vel_setpt' : self.stepper.get_vel_setpt,
                    'get_vendor_id' : self.stepper.get_vendor_id,
                    'start' : self.stepper.start,
                    'stop' : self.stepper.stop,

                    'move_by' : self.stepper.move_by,
                    'move_to_pos' : self.stepper.move_to_pos,
                    #'set_dir_setpt' : self.stepper.set_dir_setpt,
                    #'set_pos_setpt' : self.stepper.set_pos_setpt,
                    #'set_vel_and_dir' : self.stepper.set_vel_and_dir,
                    #'set_vel_setpt' : self.stepper.set_vel_setpt,
                    'soft_ramp_to_pos' : self.stepper.soft_ramp_to_pos,
                    'soft_ramp_to_vel' : self.stepper.soft_ramp_to_vel,
                    
                    # Configuration
                    #'set_enable' : self.stepper.set_enable,
                    #'set_mode' : self.stepper.set_mode,
                    #'set_pos_vel' : self.stepper.set_pos_vel,
                    #'set_status' : self.stepper.set_status,
                    #'set_zero_pos' : self.stepper.set_zero_pos,
                     }
    def set_zero_pos(self,zero_pos):
    def set_status(self, status):
    def set_enable(self,enable):
    def soft_ramp_to_vel(self,vel,dir,accel,dt=0.1):
    def soft_ramp_to_pos(self,pos,accel,pos_vel=None,dt=0.1):
        
                
        #################################################
        # Init motor
        sernum='0.0.' + self.idMotor
        gr=7.2
        clkdir_mult=1
        ind_per_rev=12800
        vel_max=30000
        vel_min=0

        if self.dummy is False:
            import stepper_motors
            self.stepper = stepper_motors.StepperMotor(sernum=sernum, gr=gr, clkdir_mult=clkdir_mult, ind_per_rev=ind_per_rev, vel_max=vel_max, vel_min=vel_min)
        #################################################
        
        # latest motor states  
        self.pos = 0
        self.vel = 0
        
        self.callbackSetControllerGains()
        self.limit_lo = -1000
        self.limit_hi = 1000
        self.limit_buffer = 0.2
        self.direction = 0
        
        
        ################################################
        # ROS stuff
        
        node = self.idMotor + '_atmel_com'
        rospy.init_node(node, anonymous=True)
        self.last_time = rospy.get_time()
        
        # Subscribe to ROS topics
        rospy.Subscriber('topicSetConfig', msgNSSConfig, self.callbackSetConfig)
        rospy.Subscriber('topicSetPositionMode', msgSetPositionMode, self.callbackSetPositionMode)
        rospy.Subscriber('topicSetVelocityMode', msgSetVelocityMode, self.callbackSetVelocityMode)
        rospy.Subscriber('topicSetPosition', msgSetPosition, self.callbackSetPosition)
        rospy.Subscriber('topicSetVelocity', msgSetVelocity, self.callbackSetVelocity)
        rospy.Subscriber("topicSetControllerGains", Bool, self.callbackSetControllerGains)

        # Publish on ROS topics
        stTopicConfig = self.idMotor + '_config'
        self.topicConfig = rospy.Publisher(stTopicConfig, msgMotorConfig)
        
        #################################################
        print 'Spinning...'
        rospy.spin()


    def callbackSetPositionMode(selfself, data):
        if data.id is self.idMotor:
            self.ss.setmode(POSITION_MODE)
            self.ss.set_pos_vel(data.velocity)
            
        
    def callbackSetVelocityMode(selfself, data):
        if data.id is self.idMotor:
            self.ss.setmode(VELOCITY_MODE)
        
        
    def callbackSetVelocity(self, data):
        if data.id is self.idMotor:
            if N.sign(self.last['vel'] * data.vel) <= 0:
                 self.ss.set_dir_setpt (N.sign(data.vel))
                 
            if self.units is UNITS_STEPS:
                vel = data.vel
            else:
                vel = data.vel * self.countsPerRev / twopi
                
            self.ss.set_vel_setpt(vel)
            self.last['vel'] = data.vel
        
        
    def callbackSetPosition(self, data):
        if data.id is self.idMotor:
            self.ss.set_pos_setpt(data.pos)
            
    
    def callbackSetControllerGains(self, data=True):
        # motor control characteristics 
        if not rospy.has_param('gain'):
            print 'Setting default parameters'
            rospy.set_param('gain', 5)
            rospy.set_param('damping', 0.5)
            rospy.set_param('accel', 0.5)
            rospy.set_param('minaccel','.01')
        
        self.damping = rospy.get_param('damping')
        self.gain = rospy.get_param('gain')
        self.max_accel = rospy.get_param('accel')
        self.min_accel = rospy.get_param('minaccel')

            

    def callbackCmd(self, data):
    
        if self.STOP_CMD is True:
            return
    
        if self.STOP_CMD is False:
            
            des_pos = data.pos
            des_vel = data.vel
            if 1:
                if des_pos < self.limit_lo: 
                    print 'HIT LO LIMIT, STOPPING!'
                    des_pos = self.limit_lo
                if des_pos > self.limit_hi: 
                    print 'HIT HI LIMIT, STOPPING!'
                    des_pos = self.limit_hi
                 
            #print self.pos
            
            if self.dummy is True:
                time0 = rospy.get_rostime()
                #print self.pos, des_pos
                self.topicConfig.publish(msgMotorConfig(self.pos, self.latency, time0))
                # controller:
                vel_des = self.gain*(des_pos-self.pos) 
                    
                # proposed acceleration:
                accel = (vel_des - self.vel)
                
                # set new desired velocity
                self.vel = self.vel + (vel_des - self.vel)*np.exp(-1*np.abs(accel)*self.damping)

                now = rospy.get_rostime()
                self.pos = self.vel*(now-self.last_time) + self.pos            
                self.last_time = now
                self.latency = now-time0
                
            
            if self.dummy is False:
                
                # try input shaping. look at Teel's work
            
                time0 = rospy.get_rostime()
                dt = 0.03
                self.pos = self.stepper.getpos()
                self.topicConfig.publish(msgMotorConfig(self.pos, self.latency, time0))
                
                # discrete low pass filter on desired position (wikipedia)
                self.alpha = dt / (self.damping + dt)
                self.des_pos_filtered = self.alpha*des_pos + (1.-self.alpha)*self.des_pos_filtered
                self.des_vel_filtered = self.alpha*des_vel + (1.-self.alpha)*self.des_vel_filtered
                
                
                # controller:
                #vel_des = self.gain*(des_pos-self.pos) + des_vel
                vel_des = self.gain*(self.des_pos_filtered-self.pos) + self.des_vel_filtered
                     
                # proposed acceleration:
                accel = (vel_des - self.vel)
                #print self.pos, des_pos, vel_des, accel
                
                ## old damping method ##
                # set new desired velocity
                # damping should be less than one if acceleration is very large, =1 is small
                #damp_factor = np.exp(np.abs(accel))*self.damping-self.damping
                #if damp_factor > 1.0:
                #    damp_factor = 0.8
                #
                
                if 1:
                    if np.abs(accel) > self.max_accel:# and (des_pos-self.pos) > 2.*np.pi/180.:
                        self.vel += self.max_accel*np.sign(accel)
                    else:
                        self.vel = vel_des
                
                self.vel = vel_des
                print self.des_pos_filtered-self.pos, vel_des, des_vel
                
                #self.vel = self.vel + (vel_des - self.vel)*(1-damp_factor)
                
                if self.pos < self.limit_lo: 
                    if self.vel < 0:
                        print 'stopping!'
                        self.vel = 0                    
                if self.pos > self.limit_hi: 
                    if self.vel > 0:
                        print 'stopping!'
                        self.vel = 0
                                    
                change_direction = 1
                if np.sign(self.vel) == self.direction:
                    change_direction = 0
                self.direction = np.sign(self.vel)
            
                #print motorid, m_control, m_current_vel, vel_des, accel, 'latency: ', latency
                self.stepper.setvel(self.vel, change_direction = change_direction)
                #print des_pos, self.pos, self.vel, des_vel
                self.lasttime = rospy.get_rostime()
                self.latency = self.lasttime.secs-time0.secs
                

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--id", type="str", dest="idMotor", default='C',
                        help="motor id, ie. A, B, C, etc.")
    parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
                        help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    parser.add_option("--zerostart", action="store_true", dest="zerostart", default=False,
                        help="zerostart uses stepper_motors.zerostart for zeroing routine")
    (options, args) = parser.parse_args()
    
    rosSS = RosSimpleStep(options.idMotor, dummy=options.dummy, zerostart=options.zerostart)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
