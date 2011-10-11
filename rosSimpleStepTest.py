#!/usr/bin/env python
import roslib; roslib.load_manifest('rosSimpleStep')
import rospy
from std_msgs.msg import *
#from ros_flydra.msg import *
from actuator_msgs.msg import msgActuatorParameters, msgSetPositionMode, msgSetVelocityMode, msgSetPosition, msgSetVelocity, msgSetZero
from actuator_msgs.srv import srvReset
from sensor_msgs.msg import JointState
import simple_step
import time
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
    
    def __init__(self, idMotor, units='radians'):
        pdb.set_trace()
        self.idMotor = idMotor
        self.units = units
        #self.modeNode = mode # Mode of RosSimpleStep commands is determined by fields in the JointState msg.
        self.modeSS = 'position'
        
        ################################################
        # ROS Stuff
        rospy.init_node('rosSimpleStep', anonymous=True)
        
        # Publish and Subscribe
        rospy.Subscriber('msgActuatorParameters', msgActuatorParameters, self.callbackActuatorParameters)
        rospy.Subscriber('msgSetZero', msgSetZero, self.callbackSetZero)
        rospy.Subscriber('msgSetPositionMode', msgSetPositionMode, self.callbackSetPositionMode)
        rospy.Subscriber('msgSetVelocityMode', msgSetVelocityMode, self.callbackSetVelocityMode)
        rospy.Subscriber('joint_states', JointState, self.callbackJointState)
        rospy.Subscriber('msgSetVelocity', msgSetVelocity, self.callbackSetVelocity)
        #rospy.Subscriber('msgSetControllerGains', Bool, self.callbackSetControllerGains)
        
        self.msgActuatorParameters = rospy.Publisher('msgActuatorParameters', msgActuatorParameters)
        rospy.Service(self.idMotor+'srvReset', srvReset, self.callbackReset)
        
        rospy.on_shutdown(self.callbackOnShutdown)
        
                
        #################################################
        # Initialize the USB key.
        self.ss = simple_step.Simple_Step(serial_number='0.0.'+self.idMotor)


        #################################################
        self.STOP_CMD = False
        self.countsPerRev = 4000 #12800
        self.countsPerRadian = self.countsPerRev / (2*N.pi)

        self.limitHi = 2*N.pi
        self.limitLo = -2*N.pi
        self.limitBuffer = 0.2

        self.posHome = 0.0
        self.posActual = 0.0
        self.posFiltered = 0.0
        self.posLast = 0.0
        
        self.velMax = 2.0*N.pi # one turn per second
        self.velMin = 0.0
        self.velDefault = 4#N.pi / 2.0 # quarter turn per second
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
        self.ss.set_vel_and_dir(0, 0)
        rospy.loginfo('1 get_pos() returns %d', self.ss.get_pos())
        self.ss.set_zero_pos(int(round(self.ss.get_pos())))
        rospy.loginfo('2 get_pos() returns %s', self.ss.get_pos())
        self.setZero(0)
        rospy.loginfo('3 get_pos() returns %s', self.ss.get_pos())
        
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
     
        rospy.loginfo ('Initialized '+self.idMotor)
        
        

    def callbackReset(self, srv):
        pdb.set_trace()
        if srv.id is self.idMotor:
            self.ss.stop()
#            self.ss.set_ext_int(1)  # Enable the 'home' switch
#            self.ss.set_vel_and_dir(self.velDefault, srv.direction)
#            while (self.ss.get_status() is not 'stopped'):
#                time.sleep(0.1)
#            
#            self.ss.set_ext_int(0)  # Disable the 'home' switch
#            self.posHome = int(round(self.ss.get_pos()))
#            self.ss.set_zero_pos(self.posHome)
#            self.posHome = int(round(self.ss.get_pos()))
            return True
        else:
            return False


    def callbackActuatorParameters(self, msg):
        pdb.set_trace()
        k = -1
        for id in msg.id:    # Find our joint in the joint list.
            k = k+1
            if id==self.idMotor:
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


    def callbackSetZero(self, msg):
        pdb.set_trace()
        k = -1
        for id in msg.id:    # Find our joint in the joint list.
            k = k+1
            if id==self.idMotor:
                rospy.loginfo('- get_pos() returns %s', self.ss.get_pos())
                self.setZero(msg.position[k])
                rospy.loginfo('+ get_pos() returns %s', self.ss.get_pos())
        

    # setZero() - Set the zero position of the motor.          
    def setZero(self,posNewZero):
        pdb.set_trace()
        
        self.ss.stop()
        
        # posHome units may be radians or counts.
        self.posHome = 0.0 #(posNewZero-self.posActual)
            
        
        # posNewZero units may be radians or counts.
        if self.units is 'radians':
            self.ss.set_zero_pos(int(round(posNewZero * self.countsPerRadian)))
        else:
            self.ss.set_zero_pos(int(round(posNewZero)))
        
        # posActual units may be radians or counts.
        if self.units is 'radians':
            self.posActual = self.ss.get_pos() / self.countsPerRadian
        else:
            self.posActual = self.ss.get_pos()

        
        rospy.loginfo("(%s) cb: posActual=%s, get_pos()=%s, posNewZero=%s", self.idMotor, self.posActual, self.ss.get_pos(), posNewZero)


#    def callbackSetPositionMode(self, msg):
#        pdb.set_trace()
#        k = -1
#        for id in msg.id:    # Find our joint in the joint list.
#            k = k+1
#            if id==self.idMotor:
#                self.mode = 'position'
#                #self.ss.set_mode('position')
#                #self.ss.set_pos_vel(msg.vel[k])
#            
#        
#    def callbackSetVelocityMode(self, msg):
#        pdb.set_trace()
#        k = -1
#        for id in msg.id:    # Find our joint in the joint list.
#            k = k+1
#            if id==self.idMotor:
#                self.mode = 'velocity'
#                #self.ss.set_mode('velocity')
        
        
    def callbackSetVelocity(self, msg):
        pdb.set_trace()
        k = -1
        for id in msg.id:    # Find our joint in the joint list.
            k = k+1
            if id==self.idMotor:
                if self.velLast * msg.vel[k] <= 0:
                     self.ss.set_dir_setpt (N.sign(msg.vel[k]))
                     
                if self.units is 'radians':
                    vel = msg.vel[k] * self.countsPerRadian
                else:
                    vel = msg.vel[k]
                    
                self.ss.set_vel_setpt(vel)
                self.velLast = msg.vel[k]
        
        
    def callbackJointState(self, msg):
        pdb.set_trace()
        k = -1
        for id in msg.name:    # Find our joint in the joint list.
            k = k+1
            if id==self.idMotor:
                    
                # Convert message units.  msg units are always radians.
                if self.units is not 'radians':
                    try:
                        posCmd = msg.position[k] * self.countsPerRadian
                        posCmd = N.clip(self.posCmd, self.limitLo*self.countsPerRadian, self.limitHi*self.countsPerRadian)
                    except:
                        posCmd = None
                    try:
                        velCmd = msg.velocity[k] * self.countsPerRadian
                    except:
                        velCmd = 0
                else: # units=='radians'
                    try:
                        posCmd = msg.position[k]
                        posCmd = N.clip(self.posCmd, self.limitLo, self.limitHi)
                    except:
                        posCmd = None
                    try:
                        velCmd = msg.velocity[k]
                    except:
                        velCmd = 0
            
                            
                if posCmd is not None:
                    velDes = self.VelCmdFromPosVelCmd (posCmd, velCmd)            
                else:
                    velDes = velCmd
                    rospy.loginfo("(%s) velDes=%s", self.idMotor, velDes)
                    
                    
                # Limit the motor position
                if self.posActual < self.limitLo: 
                    if self.velDes < 0:
                        rospy.loginfo("pos<limitLo, Stopping!")
                        self.velDes = 0
                if self.posActual > self.limitHi: 
                    if self.velDes > 0: 
                        rospy.loginfo("pos>limitHi, Stopping!")
                        self.velDes = 0
                                    
                # Motor Direction
                bDirChange = True
                if N.sign(self.velDes) == N.sign(self.velLast):
                    bDirChange = False
                    
                if N.sign(self.velDes) < 0:
                    self.direction = 1
                else:
                    self.direction = 0
                
                self.velDes = N.abs(self.velDes)
            
                self.timeCur = rospy.get_rostime()
                self.dt = float(self.timeCur.nsecs - self.timeLast.nsecs) * 10E-9
                self.timeLast = self.timeCur
                self.velLast = self.velDes
                self.posLast = self.posActual

                #self.velDes = 1
                
                # Send command to motor.
                #rospy.loginfo ("(%s) rss self.posCmd=%s", self.idMotor, self.posCmd)#, posError=%s, vel=%s, dir=%s", self.posCmd, self.posError, self.velDes, self.direction)
                #if bDirChange: 
                #    self.ss.set_dir_setpt(self.direction)
                #self.ss.set_vel_setpt(self.velDes)
                self.ss.set_vel_and_dir(self.velDes, self.direction)
                self.iStep = self.iStep+1
                #rospy.loginfo("(%s) cb: Set velDes=%s, dir=%s", self.idMotor, self.velDes, self.direction)
                    
                
            
    
    def callbackSetControllerGains(self, msg=True):
        pdb.set_trace()
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

            

    def callbackCmd(self, msg):
        pdb.set_trace()
    
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
                
                
    def callbackOnShutdown(self):
        pdb.set_trace()
        rospy.loginfo ('(%s) Stopping, ss.get_serial_number()=%s', self.idMotor, self.ss.get_serial_number())
        #time.sleep(N.random.rand())
        self.ss.stop()
        rospy.loginfo ('%s 1', self.idMotor)            
        self.ss.set_mode('position')
        rospy.loginfo ('%s 2', self.idMotor)            
        #time.sleep(0.1)
        #if self.units == 'radians':
        #    self.ss.set_pos_vel(self.velDefault * self.countsPerRadian)
        #else:
        #    self.ss.set_pos_vel(self.velDefault)
            
        #time.sleep(0.1)
        self.ss.set_pos_setpt(self.posHome)
        rospy.loginfo("(%s) cb: Goto homepos=%s", self.idMotor, self.posHome)
        time.sleep(4.0)
        self.ss.stop()            
        rospy.loginfo ('(%s) Stopped', self.idMotor)
                
    def VelCmdFromPosVelCmd(self, posCmd, velCmd): 
        if self.units=='radians':
            self.posActual = self.ss.get_pos() / self.countsPerRadian
        else:
            self.posActual = self.ss.get_pos()
        
        
        # Low-pass filter on desired position (wikipedia)
        #alpha = self.dt / (self.Kd + self.dt)
        #self.posCmdFiltered = alpha*posCmd + (1.-alpha)*self.posCmdFiltered
        #self.velCmdFiltered = alpha*velCmd + (1.-alpha)*self.velCmdFiltered
        
        self.posError = posCmd - self.posActual
        self.posErrorD = self.posError - self.posErrorLast
        self.posErrorI += self.posError
        
        # Control the velocity to track position:
        velDes = self.Kp*self.posError + self.Kd*self.posErrorD + self.Ki*self.posErrorI + velCmd
        rospy.loginfo("(%s) posA=%04f, posC=%04f, eP=%04f, eD=%04f, eI=%04f, velDes=%04f, dir=%s", self.idMotor, self.posActual, posCmd, self.posError, self.posErrorD, self.posErrorI, self.velDes, self.direction)

        return velDes
    
    
    def main(self):
        pdb.set_trace()
        rospy.spin()
                
                
                
rospy.loginfo ("rosSimpleStep name=%s", __name__)
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--id", type="str", dest="idMotor", default='A',
                        help="motorid=A|B|C|etc.")
    #parser.add_option("--mode", type="str", dest="mode", default='position',
    #                    help="mode=position|velocity")
    #parser.add_option("--dummy", action="store_true", dest="dummy", default=False,
    #                    help="with dummy = True, will not attempt to talk to controller, but will return false motor values")
    (options, args) = parser.parse_args()

    node = RosSimpleStep(options.idMotor)
    rospy.loginfo("rosSimpleStep - postinit")
    node.main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
