#!/usr/bin/env python
import roslib; roslib.load_manifest('rosSimpleStep')
import rospy
from std_msgs.msg import *
#from ros_flydra.msg import *
from actuator_msgs.msg import msgActuatorParameters, msgSetPositionMode, msgSetVelocityMode, msgSetPosition, msgSetVelocity, msgSetZero
from actuator_msgs.srv import srvReset
from sensor_msgs.msg import JointState
import simple_step
#import time
import numpy as N
from optparse import OptionParser
import string
import pdb


# Integer values for operating modes - used in usb set/get
#VELOCITY_MODE = 0
#POSITION_MODE = 1

# Integer values for directions - used in usb set/get
POSITIVE = 0
NEGATIVE = 1

# Status Constants
RUNNING = 1
STOPPED = 0

# Enable Constants
ENABLED = 1
DISABLED = 0


class RosSimpleStep:

    ##############################    
    # __init() - Initialization for class RosSimpleStep.  Called when instantiated.
    # 
    def __init__(self, nameMotor=None, units='radians'):
        #pdb.set_trace()
        self.nameMotor = nameMotor
        self.units = units
        #self.modeNode = mode # Mode of RosSimpleStep commands is determined by fields in the JointState msg.
        self.modeSS = 'position'
        
        ################################################
        # ROS Stuff
        rospy.init_node('rosSimpleStep', anonymous=True)
        rospy.sleep(1)
        rospy.loginfo ('(%s) Starting', self.nameMotor)
        
        # Publish and Subscribe
        rospy.Subscriber('msgActuatorParameters', msgActuatorParameters, self.callbackActuatorParameters)
        rospy.Subscriber('msgSetZero', msgSetZero, self.callbackSetZero)
        #rospy.Subscriber('msgSetPositionMode', msgSetPositionMode, self.callbackSetPositionMode)
        #rospy.Subscriber('msgSetVelocityMode', msgSetVelocityMode, self.callbackSetVelocityMode)
        rospy.Subscriber('joint_states', JointState, self.callbackJointState)
        rospy.Subscriber('msgSetVelocity', msgSetVelocity, self.callbackSetVelocity)
        #rospy.Subscriber('msgSetControllerGains', Bool, self.callbackSetControllerGains)
        
        self.msgActuatorParameters = rospy.Publisher('msgActuatorParameters', msgActuatorParameters)
        rospy.Service(self.nameMotor+'srvReset', srvReset, self.callbackReset)
        
        rospy.on_shutdown(self.callbackOnShutdown)
        
                
        #################################################
        # Initialize the USB key.
        self.ss = simple_step.Simple_Step(serial_number='0.0.'+self.nameMotor)


        #################################################
        self.STOP_CMD = False
        self.countsPerRev = 4000 #12800
        self.countsPerRadian = self.countsPerRev / (2*N.pi)

        self.limitHi = 2*N.pi
        self.limitLo = -2*N.pi
        self.limitBuffer = 0.2

        self.posIndex = 0.0
        self.posActual = 0.0
        self.posFiltered = 0.0
        self.posLast = 0.0
        
        self.velMax = 2.0*N.pi # one turn per second
        self.velMin = 0.0
        self.velDefault = N.pi / 4.0 # eighth turn per second
        self.velFiltered = 0.0
        self.velDes = 0.0
        self.velLast = 0.0
        
        self.direction = 0
        self.dirLast = 0

        self.iStep = 0
        self.dt = 0.03
        
        self.posErrorLast = 0.0;
        self.posErrorI = 0.0;
        self.timeLast = rospy.get_rostime()

        
        #################################################
        # Stop the motor, and set current position as zero
        self.ss.stop()
        self.ss.set_mode(self.modeSS) # USBkey mode is separate from ros node mode.
        #self.ss.set_vel_and_dir(0, 0)
        rospy.loginfo('(%s) 1 get_pos() returns %d', self.nameMotor, self.ss.get_pos())
        self.ss.set_zero_pos(int(round(self.ss.get_pos())))
        rospy.loginfo('(%s) 2 get_pos() returns %s', self.nameMotor, self.ss.get_pos())
        self.setZero(0)
        rospy.loginfo('(%s) 3 get_pos() returns %s', self.nameMotor, self.ss.get_pos())
        
        if self.units == 'radians':
            self.ss.set_pos_vel(self.velDefault * self.countsPerRadian)
        else:
            self.ss.set_pos_vel(self.velDefault)
        
                    
        ################################################
        # Controller gains  (10,-0.001,4 are good for bare motors)
        #self.callbackSetControllerGains()
        self.Kp = 20.0
        self.Kd = -0.1 #-0.0001
        self.Ki =  0.0
     
        rospy.loginfo ('(%s) Initialized. ', self.nameMotor)
        
        

    ##############################    
    # callbackReset() - Service callback to find the "home" switch, then go to position (x,y) = (0,0).
    #
    # srv.id        - The actuator identifier.
    # srv.direction - The direction to move to find home.
    #
    def callbackReset(self, srv):
        #pdb.set_trace()
        if srv.name is self.nameMotor:
            self.ss.stop()
#            self.ss.set_ext_int(1)  # Enable the 'home' switch
#            self.ss.set_vel_and_dir(self.velDefault, srv.direction)
#            while (self.ss.get_status() is not 'stopped'):
#                rospy.sleep(0.1)
#            
#            self.ss.set_ext_int(0)  # Disable the 'home' switch
#            self.posIndex = int(round(self.ss.get_pos()))
#            self.ss.set_zero_pos(self.posIndex)
#            self.posIndex = int(round(self.ss.get_pos()))
            return True
        else:
            return False


    ##############################
    # callbackActuatorParameters() - Message callback to set actuator parameters.
    # 
    # msg.name[]           - The actuator identifier, one entry per actuator.
    # msg.countsPerRev[] - Encoder resolution.
    # msg.units[]        - 'radians' or 'counts', the units of angles stored in this ROS node.
    # msg.velMin[]       - Minimum velocity allowed.
    # msg.velMax[]       - Maximum velocity allowed.
    # msg.velDefault[]   - Default velocity to use when positioning.
    #
    def callbackActuatorParameters(self, msg):
        #pdb.set_trace()
        k = -1
        for name in msg.name:    # Find our joint in the joint list.
            k = k+1
            if name==self.nameMotor:
                if msg.countsPerRev[k] is not None:
                    self.countsPerRev = msg.countsPerRev[k]
                    self.countsPerRadian = self.countsPerRev / (2*N.pi)
                    
                if msg.units[k] is not None:
                    self.units = msg.units[k]
                    
                if msg.velMax[k] is not None:
                    self.velMax = msg.velMax[k]
                    
                if msg.velMin[k] is not None:
                    self.velMin = msg.velMin[k]
                    
                if msg.velDefault[k] is not None:
                    self.velDefault = msg.velDefault[k]


    ##############################
    # callbackSetZero() - Message callback to set the current position of the actuator as zero.
    #
    # msg.name[]   - The actuator identifier, one entry per actuator.
    #
    def callbackSetZero(self, msg):
        #pdb.set_trace()
        k = -1
        for name in msg.name:    # Find our joint in the joint list.
            k = k+1
            if name==self.nameMotor:
                rospy.loginfo('(%s) get_pos()- returns %s', self.nameMotor, self.ss.get_pos())
                self.setZero(msg.position[k])
                rospy.loginfo('(%s) get_pos()+ returns %s', self.nameMotor, self.ss.get_pos())
        

    ##############################
    # setZero() - Set the zero position of the motor at the specified location.
    #          
    # posNewZero - The new zero position, in current coordinates.
    #
    def setZero(self,posNewZero):
        #pdb.set_trace()
        
        self.ss.stop()
        self.posIndex = -posNewZero # BUG: why?
        self.ss.set_zero_pos(self.countFromUnits(posNewZero))
        #self.posActual = self.unitsFromCount(self.ss.get_pos())
        #rospy.loginfo("(%s) cb: posActual=%s, get_pos()=%s, posNewZero=%s", self.nameMotor, self.posActual, self.ss.get_pos(), posNewZero)


    ##############################
    # callbackSetPositionMode() - Message callback to put this ROS node into position mode.
    #
    # msg.name[]  - The actuator identifier, one entry per actuator.
    #
#    def callbackSetPositionMode(self, msg):
#        k = -1
#        for name in msg.name:    # Find our joint in the joint list.
#            k = k+1
#            if name==self.nameMotor:
#                self.mode = 'position'
#                #self.ss.set_mode('position')
#                #self.ss.set_pos_vel(msg.vel[k])
#            
#        
    ##############################
    # callbackSetVelocityMode() - Message callback to put this ROS node into velocity mode.
    #
    # msg.name[]  - The actuator identifier, one entry per actuator.
    #
#    def callbackSetVelocityMode(self, msg):
#        #pdb.set_trace()
#        k = -1
#        for name in msg.name:    # Find our joint in the joint list.
#            k = k+1
#            if name==self.nameMotor:
#                self.mode = 'velocity'
#                #self.ss.set_mode('velocity')
        
        
    ##############################
    # callbackSetVelocity() - Message callback to set the actuator velocity.
    #
    # msg.name[]  - The actuator identifier, one entry per actuator.
    # msg.vel[] - The new velocity of the motor, velMin < vel < velMax.
    #
    def callbackSetVelocity(self, msg):
        #pdb.set_trace()
        k = -1
        for name in msg.name:    # Find our joint in the joint list.
            k = k+1
            if name==self.nameMotor:
                if self.velLast * msg.vel[k] <= 0:
                     self.ss.set_dir_setpt (N.sign(msg.vel[k]))
                     
                vel = unitsFromRadians(msg.vel[k])
                self.ss.set_vel_setpt(vel)
                self.velLast = msg.vel[k]
        
        
    ##############################
    # callbackJointState() - Message callback for JointState updates.  Sends command to actuator.
    #
    # msg.name[]           - The actuator identifier, one entry per actuator.
    # msg.position[]       - Desired actuator position.
    # msg.velocity[]       - Desired actuator velocity.
    #
    def callbackJointState(self, msg):
        #pdb.set_trace()
        k = -1
        for name in msg.name:    # Find our joint in the joint list.
            k = k+1
            if name==self.nameMotor:
                
                # Convert message radians to node units.
                if self.units=='radians':
                    try:
                        posDes = msg.position[k]
                        posDes = N.clip(posDes, self.limitLo, self.limitHi)
                    except:
                        posDes = None
                    try:
                        velDes = msg.velocity[k]
                    except:
                        velDes = 0
                else:
                    try:
                        posDes = msg.position[k] * self.countsPerRadian
                        posDes = N.clip(posDes, self.limitLo*self.countsPerRadian, self.limitHi*self.countsPerRadian)
                    except:
                        posDes = None
                    try:
                        velDes = msg.velocity[k] * self.countsPerRadian
                    except:
                        velDes = 0


                # Get the time delta.
                self.timeCur = rospy.get_rostime()
                dsecs = self.timeCur.secs - self.timeLast.secs
                dnsecs = self.timeCur.nsecs - self.timeLast.nsecs
                self.dt = float(dsecs * 1E9 + dnsecs) * 1E-9
                

                # Compute position and velocity commands to reach desired position & velocity.
                posCmd = self.posCmdFromPosVelDes (posDes, velDes)
                velCmd = self.velCmdFromPosVelDes (posDes, velDes)
                    
                # Convert velocity to magnitude & direction
                bDirChange = True
                if N.sign(velCmd) == N.sign(self.velLast):
                    bDirChange = False
                    
                if N.sign(velCmd) < 0:
                    direction = 1
                else:
                    direction = 0
                
                velCmd = N.abs(velCmd)
                
                
                if self.modeSS=='position':
                    #rospy.loginfo("(%s) posCmd=%s, posActual=%s, ssmode=%s", self.nameMotor, self.countFromUnits(posCmd), self.countFromUnits(self.posActual), self.ss.get_mode())
                    self.ss.set_pos_setpt(self.countFromUnits(posCmd))
                    self.ss.start()
                    rospy.loginfo("(%s) posCmd=%s", self.nameMotor, posCmd)
                else:                                        
                    # Send command to motor.
                    #rospy.loginfo ("(%s) rss posCmd=%s", self.nameMotor, posCmd)#, posError=%s, vel=%s, dir=%s", posCmd, self.posError, velCmd, direction)
                    #if bDirChange: 
                    #    self.ss.set_dir_setpt(direction)
                    #self.ss.set_vel_setpt(self.velDes)
                    rospy.loginfo("(%s) velCmd=%s, dir=%s", self.nameMotor, velCmd, direction)
                    self.ss.set_vel_and_dir(velCmd, direction)
                    
                    
                rospy.loginfo('(%s) %0.3f Hz', self.nameMotor, 1.0/self.dt)

                self.timeLast = self.timeCur
                self.velLast = velCmd
                self.iStep = self.iStep+1
                #rospy.loginfo("(%s) cb: Set velDes=%s, dir=%s", self.nameMotor, self.velDes, direction)
                    
        
                
    ##############################
    # msg.name[]           - The actuator identifier, one entry per actuator.
    def callbackSetControllerGains(self, msg=True):
        #pdb.set_trace()
        # motor control characteristics 
        if not rospy.has_param('Kp'):
            print 'Setting default parameters'
            rospy.set_param('Kp', 8.0)
            rospy.set_param('Kd', -0.005)
            rospy.set_param('Ki', 0.0)
            rospy.set_param('accelMax', 0.5)
            rospy.set_param('accelMin', 0.01)
        
        self.Kd = rospy.get_param('Kd')
        self.Kp = rospy.get_param('Kp')
        self.Ki = rospy.get_param('Ki')
        
        self.accelMax = rospy.get_param('accelMax')
        self.accelMin = rospy.get_param('accelMin')

     # msg.name[] - 
           

    ##############################
    # msg.name[]           - The actuator identifier, one entry per actuator.
    def callbackCmd(self, msg):
        #pdb.set_trace()
    
        if self.STOP_CMD is True:
            return
        else:
            des_pos = msg.pos
            des_vel = msg.vel
            if self.dummy is True:
                #print self.pos, des_pos
                #self.msgConfig.publish(msgMotorConfig(self.pos, self.dt, time0))
                # controller:
                vel_des = self.Kp * (des_pos - self.pos) 
                    
                # proposed acceleration:
                accel = (vel_des - self.vel)
                
                # set new desired velocity
                self.vel = self.vel + (vel_des - self.vel)*N.exp(-1*N.abs(accel)*self.Kd)

                now = rospy.get_rostime()
                self.dt = float(now.nsecs - self.timeLast.nsecs) * 10E-9
                self.pos = self.vel*dt + self.pos            
                self.timeLast = now
                
                
    ##############################
    def callbackOnShutdown(self):
        #pdb.set_trace()
        rospy.loginfo ('(%s) Stopping, ss.get_serial_number()=%s', self.nameMotor, self.ss.get_serial_number())
        #rospy.sleep(N.random.rand())
        #self.ss.stop()
        rospy.loginfo ('%s 1', self.nameMotor)            
        self.ss.set_mode('position')
        rospy.loginfo ('%s 2', self.nameMotor)            
        #rospy.sleep(0.1)
        #if self.units == 'radians':
        #    self.ss.set_pos_vel(self.velDefault * self.countsPerRadian)
        #else:
        #    self.ss.set_pos_vel(self.velDefault)
            
        #rospy.sleep(0.1)
        self.ss.set_pos_setpt(self.countFromUnits(self.posIndex))
        rospy.loginfo("(%s) cb: Goto posIndex=%s", self.nameMotor, self.posIndex)
        rospy.sleep(4.0)
        self.ss.stop()            
        rospy.loginfo ('(%s) Stopped', self.nameMotor)

                
    ##############################
    def countFromUnits(self, val):            
        if self.units=='radians':
            return int(round(val * self.countsPerRadian))
        else:
            return int(val)
        
    ##############################
    def radiansFromUnits(self, val):            
        if self.units=='radians':
            return val
        else:
            return val / self.countsPerRadian
    
    ##############################
    def unitsFromCount(self, count):            
        if self.units=='radians':
            return count / self.countsPerRadian
        else:
            return val
        
    ##############################
    def unitsFromRadians(self, radians):            
        if self.units=='radians':
            return radians
        else:
            return radians * self.countsPerRadian
        
        
    ##############################
    # Computes the position command needed to reach the desired position and velocity in dt.
    # Currently this function is simplistic, and just returns the desired position.  Could extrapolate
    # a position based on motor speed, etc.
    def posCmdFromPosVelDes(self, posDes, velDes):
        if posDes is not None:
            posCmd = posDes
        else:
            posCmd = 0
            
        return posCmd  

    
    ##############################
    # Computes the velocity command needed to reach the desired position and velocity in self.dt.
    def velCmdFromPosVelDes(self, posDes, velDes): 
        if posDes is None:  # If no position was specified, there's no position correction.
            velPos = 0
        else:               # If a position was specified, then control toward that position.
            # Low-pass filter on desired position (wikipedia)
            #alpha = self.dt / (self.Kd + self.dt)
            #self.posDesFiltered = alpha*posDes + (1.-alpha)*self.posDesFiltered
            #self.velDesFiltered = alpha*velDes + (1.-alpha)*self.velDesFiltered
            self.posActual = self.unitsFromCount(self.ss.get_pos())
            self.posError = posDes - self.posActual
            self.posErrorD = self.posError - self.posErrorLast
            self.posErrorI += self.posError
            velPos = self.Kp*self.posError + self.Kd*self.posErrorD + self.Ki*self.posErrorI
            
        # Control the velocity to track position:
        velCmd = velPos + velDes
        #rospy.loginfo("(%s) posA=%04f, posDes=%04f, eP=%04f, eD=%04f, eI=%04f, velCmd=%04f, dir=%s", self.nameMotor, self.posActual, posDes, self.posError, self.posErrorD, self.posErrorI, velCmd, self.direction)


        # Limit the motor position
        if self.posActual < self.limitLo: 
            if velCmd < 0:
                rospy.loginfo("pos<limitLo, Stopping!")
                velCmd = 0
        if self.posActual > self.limitHi: 
            if velCmd > 0: 
                rospy.loginfo("pos>limitHi, Stopping!")
                velCmd = 0
                                    

        return velCmd
    
    
    ##############################
    def main(self):
        #pdb.set_trace()
        rospy.spin()
                
                
                
rospy.loginfo ("rosSimpleStep name=%s", __name__)
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--id", type="str", dest="nameMotor", default='A',
                        help="motorid=A|B|C|etc.")
    #parser.add_option("--mode", type="str", dest="mode", default='position',
    #                    help="mode=position|velocity")
    #parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
    #                    help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    (options, args) = parser.parse_args()

    node = RosSimpleStep(nameMotor=options.nameMotor)
    rospy.loginfo("rosSimpleStep - postinit")
    node.main()
    
    
    
    
    
    
    
    
