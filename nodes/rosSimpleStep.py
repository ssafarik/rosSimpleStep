#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('rosSimpleStep')
import rospy
import string
import numpy as N
from std_msgs.msg import *
from rosSimpleStep.srv import SrvCalibrate, SrvJointState, SrvJointStateResponse, SrvPark, SrvSetZero
from sensor_msgs.msg import JointState
import simple_step
from optparse import OptionParser

# The following settings are the best I've found for the BAI controller with the Fivebar mechanism.
# Unlisted parameters are set to their default.
# Jan 12, 2012
#
#PRM:#     Parameter                  Value         Default 
#----------------------------------------------------------------
#PRM:0:    KP                         4537740       750000
#PRM:1:    KI                         84154         35000
#PRM:2:    KPOS                       1729          15000
#PRM:11:   integral clamp             500           5000
#PRM:20:   operating mode             5             4
#PRM:26:   lowpass filter             1             0
#PRM:90:   baud rate                  38400         9600
#PRM:95:   daisy chain                1             0
#PRM:101:  fault output               1             0
#PRM:202:  filter cutoff              100.0         500.0
#PRM:204:  autotune distance          500.0         32000.0
#PRM:205:  autotune bandwidth         7.0           20.0

#PRM:#     Parameter                  Value         Default 
#----------------------------------------------------------------
#PRM:0:    KP                         4537740       750000
#PRM:1:    KI                         84154         35000
#PRM:2:    KPOS                       1729          15000
#PRM:11:   integral clamp             500           5000
#PRM:20:   operating mode             5             4
#PRM:26:   lowpass filter             1             0
#PRM:90:   baud rate                  38400         9600
#PRM:95:   daisy chain                1             0
#PRM:101:  fault output               1             0
#PRM:202:  filter cutoff              100.0         500.0
#PRM:204:  autotune distance          500.0         32000.0
#PRM:205:  autotune bandwidth         7.0           20.0
#
#ssafarik@flappy:~/pybai/BAI$ python cmd_line.py -a B -b 38400 read-param nondefault
#
#PRM:#     Parameter                  Value         Default 
#----------------------------------------------------------------
#PRM:0:    KP                         4537740       750000
#PRM:1:    KI                         84154         35000
#PRM:2:    KPOS                       1729          15000
#PRM:11:   integral clamp             500           5000
#PRM:20:   operating mode             5             4
#PRM:26:   lowpass filter             1             0
#PRM:90:   baud rate                  38400         9600
#PRM:94:   unit address               B             A
#PRM:95:   daisy chain                1             0
#PRM:101:  fault output               1             0
#PRM:202:  filter cutoff              100.0         500.0
#PRM:204:  autotune distance          500.0         32000.0
#PRM:205:  autotune bandwidth         7.0           20.0


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
    # __init() - Initialization for class RosSimpleStep.
    # 
    def __init__(self, name=None, id=None, units='radians'):
        self.name = name
        self.id = id
        self.units = units
        self.modeSS = 'position'
        #self.modeSS = 'velocity'
        
        self.initialized = False
        

        ################################################
        self._UpdateParameters()
     
        ################################################
        # ROS Stuff
        rospy.init_node('rosSimpleStep', anonymous=True)
        rospy.sleep(1)
        rospy.loginfo ("(%s) Starting", self.name)
        self.rate = rospy.Rate(100)
        
        # Messages and Services
        #rospy.loginfo ("Offering services %s, %s, %s, %s, %s, %s" % ('srvCalibrate_'+self.name,'srvSetPosition_'+self.name,'srvSetVelocity_'+self.name,'srvGetState_'+self.name,'srvPark_'+self.name,'srvSetZero_'+self.name))
        rospy.Service('srvCalibrate_'+self.name,        SrvCalibrate,   self.Calibrate_callback)
        rospy.Service('srvSetPosition_'+self.name,      SrvJointState,  self.SetPosition_callback)
        rospy.Service('srvSetVelocity_'+self.name,      SrvJointState,  self.SetVelocity_callback)
        rospy.Service('srvSetPositionAtVel_'+self.name, SrvJointState,  self.SetPositionAtVel_callback)
        rospy.Service('srvGetState_'+self.name,         SrvJointState,  self.GetState_callback)
        rospy.Service('srvPark_'+self.name,             SrvPark,        self.Park_callback)
        rospy.Service('srvSetZero_'+self.name,          SrvSetZero,     self.SetZero_callback)

        rospy.Subscriber('msgParameterUpdate', Bool, self.UpdateParameters_callback)
        #rospy.Subscriber('joint_states', JointState, self.JointState_callback)
        #rospy.Subscriber('msgSetVelocity', msgSetVelocity, self.SetVelocity_callback)
        
        rospy.on_shutdown(self.OnShutdown_callback)
        
                
        #################################################
        # Initialize the USB key.
        rospy.loginfo ('SS self.id=%s' % self.id)
        self.ss = simple_step.Simple_Step(serial_number=self.id)


        #################################################
        self.limitBuffer = 0.2

        self.posOrigin = 0.0
        self.posPark = 0.0
        self.posActual = 0.0
        self.posFiltered = 0.0
        self.posLast = 0.0
        
        self.velFiltered = 0.0
        self.velDes = 0.0
        self.velLast = 0.0
        
        self.direction = 0
        self.dirLast = 0

        self.iStep = 0
        self.dt = 0.01
        
        self.posErrorLast = 0.0;
        self.posErrorI = 0.0;
        self.timeLast = rospy.get_rostime()

        
        #################################################
        # Stop the motor, and set current position as zero
        #rospy.loginfo ('SS stop()')
        self.ss.stop()

        #rospy.loginfo ('SS set_mode()')
        self.ss.set_mode(self.modeSS) # USBkey mode is separate from ros node mode.
        #self.ss.set_vel_and_dir(0, 0)
        ##rospy.loginfo('(%s) 1 get_pos() returns %d', self.name, self.ss.get_pos())
        #rospy.loginfo ('SS set_zero_pos()')
        self.ss.set_zero_pos(int(round(self.ss.get_pos())))
        ##rospy.loginfo('(%s) 2 get_pos() returns %s', self.name, self.ss.get_pos())
        self._SetZero(0.0)
        ##rospy.loginfo('(%s) 3 get_pos() returns %s', self.name, self.ss.get_pos())
        
        #rospy.loginfo ('SS set_set_pos_vel()')
        self.ss.set_pos_vel(self._CountFromUnits(self.velDefault))

        #rospy.loginfo ('SS set_ext_int()')
        self.ss.set_ext_int('enabled')
        self.posCache = self.ss.get_pos()
        
                            

    ##############################    
    # Calibrate_callback() - Service callback to find the "index" switch, set the origin (dist from index) as theta=0, 
    #                        and go to the parking spot. 
    #
    # req.direction - The direction to move to find home.
    # req.posOrigin - Location of origin, from index.
    # req.posPark   - Location of parking spot, from origin.
    #
    def Calibrate_callback(self, srvCalibrate):
        rospy.loginfo ('(%s) Received Calibrate_callback(direction=%s)', self.name, srvCalibrate.direction)
        # Set current position as parking spot.
        #self.posPark = self._UnitsFromCount(self.ss.get_pos())
        self.posOrigin = srvCalibrate.posOrigin
        self.posPark = srvCalibrate.posPark
        
        #rospy.loginfo ('(%s) units(getpos()) returns %s --', self.name, self._UnitsFromCount(self.ss.get_pos()))
        
        # Move the motor toward the index switch.
        self.ss.stop()
        if srvCalibrate.findIndex:
            self.ss.set_mode('velocity')
            self.ss.set_ext_int('enabled')  # Enable the 'index' switch
            self.ss.set_vel_and_dir(self._CountFromUnits(self.velDefault), srvCalibrate.direction)
    
            # Wait until we hit the switch.
            while (self.ss.get_status() is not 'stopped'):
                self.rate.sleep()
            
        # Disable the 'index' switch
        self.ss.set_ext_int('disabled')
            
        # Get the current location.
        posIndex = self._UnitsFromCount(self.ss.get_pos())
        #rospy.loginfo ('(%s) units(getpos()) returns %s -', self.name, self._UnitsFromCount(self.ss.get_pos()))
        
        # Go back to the proper mode  
        self.ss.set_mode(self.modeSS) # USBkey mode is separate from ros node mode.

        # Set the given origin as 0.
        self._SetZero(self.posOrigin + posIndex)
        self.ss.start()
        self.initialized = True

        # Go back to the parking spot.
        self.Park()
        #rospy.loginfo ('SS (%s) Calibrated. ', self.name)
        
        #rospy.loginfo ('(%s) units(getpos()) returns %s +', self.name, self._UnitsFromCount(self.ss.get_pos()))
        return self._UnitsFromCount(self.ss.get_pos())
    


    ##############################
    # SetZero_callback() - Service callback to set the current position of the actuator as zero.
    #
    def SetZero_callback(self, req):
        #rospy.loginfo('(%s) get_pos()- returns %s', self.name, self.ss.get_pos())
        self._SetZero(req.pos)
        #rospy.loginfo('(%s) get_pos()+ returns %s', self.name, self.ss.get_pos())
        return True
        
        

    ##############################
    # SetPositionMode_callback() - Message callback to put this ROS node into position mode.
    #
    # msg.name[]  - The actuator identifier, one entry per actuator.
    #
#    def SetPositionMode_callback(self, msg):
#        k = -1
#        for name in msg.name:    # Find our joint in the joint list.
#            k = k+1
#            if name==self.name:
#                self.mode = 'position'
#                #self.ss.set_mode('position')
#                #self.ss.set_pos_vel(msg.vel[k])
#            
#        
    ##############################
    # SetVelocityMode_callback() - Message callback to put this ROS node into velocity mode.
    #
    # msg.name[]  - The actuator identifier, one entry per actuator.
    #
#    def SetVelocityMode_callback(self, msg):
#        k = -1
#        for name in msg.name:    # Find our joint in the joint list.
#            k = k+1
#            if name==self.name:
#                self.mode = 'velocity'
#                #self.ss.set_mode('velocity')
        
        
    ##############################
    # JointState_callback() - Message callback for JointState updates.  Sends command to actuator.
    #
    # msg.name[]           - The actuator identifier, one entry per actuator.
    # msg.position[]       - Desired actuator position.
    # msg.velocity[]       - Desired actuator velocity.
    #
    def JointState_callback(self, msg):
        k = -1
        for name in msg.name:    # Find our joint in the joint list.
            k = k+1
            if name==self.name and self.initialized:
                self.SetState(msg.position[k], msg.velocity[k])
                
                

    ##############################
    #
    def SetPositionAtVel_callback(self, req):
        #rospy.loginfo ('(%s) SetPosition_callback req=%s' % (self.name, req))
        self.ss.set_pos_vel(self._CountFromUnits(req.velocity))
        self.SetState(req.position, req.velocity, usecached=True)
        rv = req

        return (rv.header, rv.position, rv.velocity)

    
    ##############################
    #
    def SetPosition_callback(self, req):
        #rospy.loginfo ('(%s) SetPosition_callback req=%s' % (self.name, req))
        self.SetState(req.position, 0.0)
        rv = req

        return (rv.header, rv.position, rv.velocity)

    
    ##############################
    ##############################
    # SetVelocity_callback() - Service callback to set the actuator velocity.
    #
    # srv.vel[] - The new velocity of the motor, velMin < vel < velMax.
    #
    def SetVelocity_callback(self, req):
        #rospy.loginfo ('(%s) setVelocity pos=%s, vel=%s' % (self.name, req.position, req.velocity))
        self.SetState(None, req.velocity)
        rv = req
     
        return (rv.header, rv.position, rv.velocity)
    def SetVelocity_callback(self, req):
        if self.initialized:
            if self.velLast * req.velocity <= 0.0:
                 self.ss.set_dir_setpt (N.sign(req.velocity))
                 
            self.ss.set_vel_setpt(self._CountFromUnits(req.velocity))
            self.velLast = req.velocity
            rv = req
        else:
            rv.header = None
            rv.position = None
            rv.velocity = None
            
        return (rv.header, rv.position, rv.velocity)
        
        
    def Start_callback(self, req):
        self.ss.start()
        
    ##############################
    #
    def SetState(self, pos, vel, usecached=False):
        #rospy.loginfo ('(%s) SetState pos=%s, vel=%s' % (self.name, pos,vel))
        # Convert message radians to node units.
        if self.units=='radians':
            try:
                posDes = N.clip(pos, self.limitLo, self.limitHi)
            except:
                posDes = None
            try:
                velDes = N.clip(vel, self.velMin, self.velMax)
            except:
                velDes = 0.0
        else:
            try:
                posDes = pos * self.countsPerRadian
                posDes = N.clip(posDes, self.limitLo*self.countsPerRadian, self.limitHi*self.countsPerRadian)
            except:
                posDes = None
            try:
                velDes = vel * self.countsPerRadian
            except:
                velDes = 0.0


        # Get the time delta.
        self.timeCur = rospy.get_rostime()
        dsecs = self.timeCur.secs - self.timeLast.secs
        dnsecs = self.timeCur.nsecs - self.timeLast.nsecs
        self.dt = float(dsecs * 1E9 + dnsecs) * 1E-9
        

        # Compute position and velocity commands to reach desired position & velocity.
        posCmd = self._PosCmdFromPosVelDes (posDes, velDes)
        velCmd = self._VelCmdFromPosVelDes (posDes, velDes, usecached=usecached)
            
        # Convert velocity to magnitude & direction
        bDirChange = True
        if N.sign(velCmd) == N.sign(self.velLast):
            bDirChange = False
            
        if N.sign(velCmd) < 0.0:
            direction = 1
        else:
            direction = 0
        
        velCmd = N.abs(velCmd)
        
        
        if self.modeSS=='position':
            #rospy.loginfo("(%s) posCmd=%s, posActual=%s, ssmode=%s", self.name, self._CountFromUnits(posCmd), self._CountFromUnits(self.posActual), self.ss.get_mode())
            self.ss.set_pos_setpt(self._CountFromUnits(posCmd))
            #self.ss.start()
            #rospy.loginfo("(%s) posCmd=%s", self.name, posCmd)
        else:                                        
            # Send command to motor.
            #rospy.loginfo ("(%s) rss posCmd=%s", self.name, posCmd)#, posError=%s, vel=%s, dir=%s", posCmd, self.posError, velCmd, direction)
            #if bDirChange: 
            #    self.ss.set_dir_setpt(direction)
            #self.ss.set_vel_setpt(self.velDes)
            #rospy.loginfo("(%s) velCmd=%s, dir=%s", self.name, velCmd, direction)
            self.ss.set_vel_and_dir(self._CountFromUnits(velCmd), direction)
            
            
        #rospy.loginfo('(%s) %0.3f Hz', self.name, 1.0/self.dt)

        self.timeLast = self.timeCur
        self.velLast = velCmd
        self.iStep = self.iStep+1
        #rospy.loginfo("(%s) cb: Set velDes=%s, dir=%s", self.name, self.velDes, direction)
                    
        
                
    ##############################
    # GetState_callback() - Service callback to get position and velocity.
    #
    def GetState_callback(self, req):
        (header, position, velocity) = self.GetState()
        #rospy.loginfo ('SS state=%s, %s, %s' % (header, position, velocity))
        return (header, position, velocity)
    
    
    def GetState(self, usecached=False):
        rv = SrvJointStateResponse()
        if self.initialized:
            if not usecached:
                self.posCache = self.ss.get_pos()
            
            #rv.header.stamp=rospy.Time.now()
            rv.header.frame_id=self.name
            rv.position = self._UnitsFromCount(self.posCache)
            rv.velocity = self._UnitsFromCount(self.ss.get_vel())
        else:
            rv.header.frame_id=self.name
            rv.position = 0.0
            rv.velocity = 0.0
        
         
        return (rv.header, rv.position, rv.velocity)


    ##############################
    # msg.name[]           - The actuator identifier, one entry per actuator.
    def cbCmd(self, msg):
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
    def OnShutdown_callback(self):
        rospy.loginfo ('SS (%s) Stopping, ss.get_serial_number()=%s', self.name, self.ss.get_serial_number())
        self.ss.set_mode('position')
        self.Park()
        #self.ss.stop()            


    ##############################
    def Park_callback(self, req):
        self.Park()
        return True
    
        
    ##############################
    def Park(self):
        if self.initialized:
            self.ss.set_mode('position')
            self.ss.set_pos_setpt(self._CountFromUnits(self.posPark))
            rospy.sleep(1.0)
            self.ss.set_mode(self.modeSS)
        
    
    ##############################
    # _SetZero() - Set the zero position of the motor at the specified location.
    #          
    # posNewZero - The new zero position, in current coordinates.
    #
    def _SetZero(self,posNewZero):
        self.ss.set_zero_pos(self._CountFromUnits(posNewZero))


    ##############################
    # UpdateParameters_callback() - Message callback to set actuator parameters.
    #
    def UpdateParameters_callback(self, msg=None):
        self._UpdateParameters()
        

    ##############################
    def _UpdateParameters (self):
        self.countsPerRev = rospy.get_param('rosSimpleStep/countsPerRev', 4000.0)
        self.countsPerRadian = self.countsPerRev / (2.0*N.pi)
        
        self.limitLo = rospy.get_param('rosSimpleStep/limitLo', -2.0*N.pi)
        self.limitHi = rospy.get_param('rosSimpleStep/limitHi', 2.0*N.pi)
        self.velMin = rospy.get_param('rosSimpleStep/velMin', -2.0*N.pi)
        self.velMax = rospy.get_param('rosSimpleStep/velMax', 2.0*N.pi)
        self.velDefault = rospy.get_param('rosSimpleStep/velDefault', N.pi/2.0)
        self.Kp = rospy.get_param('rosSimpleStep/Kp', 1.0)
        self.Ki = rospy.get_param('rosSimpleStep/Ki', 0.0)
        self.Kd = rospy.get_param('rosSimpleStep/Kd', 0.0)


           
    ##############################
    def _CountFromUnits(self, val):            
        if self.units=='radians':
            return int(round(val * float(self.countsPerRadian)))
        else:
            return int(val)
        
    ##############################
    def _RadiansFromUnits(self, val):            
        if self.units=='radians':
            return val
        else:
            return val / float(self.countsPerRadian)
    
    ##############################
    def _UnitsFromCount(self, count):            
        if self.units=='radians':
            return float(count) / float(self.countsPerRadian)
        else:
            return count
        
    ##############################
    def _UnitsFromRadians(self, radians):            
        if self.units=='radians':
            return radians
        else:
            return radians * float(self.countsPerRadian)
        
        
    ##############################
    # Computes the position command needed to reach the desired position and velocity in dt.
    # Currently this function is simplistic, and just returns the desired position.  Could extrapolate
    # a position based on motor speed, etc.
    def _PosCmdFromPosVelDes(self, posDes, velDes):
        if posDes is not None:
            posCmd = posDes
        else:
            posCmd = 0
            
        return posCmd  

    
    ##############################
    # Computes the velocity command needed to reach the desired position and velocity in self.dt.
    def _VelCmdFromPosVelDes(self, posDes, velDes, usecached=False): 
        if posDes is None:  # If no position was specified, there's no position correction.
            velPos = 0
        else:               # If a position was specified, then control toward that position.
            # Low-pass filter on desired position (wikipedia)
            #alpha = self.dt / (self.Kd + self.dt)
            #self.posDesFiltered = alpha*posDes + (1.-alpha)*self.posDesFiltered
            #self.velDesFiltered = alpha*velDes + (1.-alpha)*self.velDesFiltered
            if not usecached:
                self.posCache = self.ss.get_pos()
                
            self.Kp = rospy.get_param('rosSimpleStep/Kp', 1.0)
            self.Ki = rospy.get_param('rosSimpleStep/Ki', 0.0)
            self.Kd = rospy.get_param('rosSimpleStep/Kd', 0.0)

            self.posActual = self._UnitsFromCount(self.posCache)
            self.posError = posDes - self.posActual
            self.posErrorD = self.posError - self.posErrorLast
            self.posErrorI += self.posError
            velPos = self.Kp*self.posError + self.Kd*self.posErrorD + self.Ki*self.posErrorI
            
        # Control the velocity to track position:
        velCmd = velPos + velDes
        rospy.loginfo("(%s) posA=%04f, posDes=%04f, posE=%04f, posD=%04f, posI=%04f, velCmd=%04f, dir=%s", 
                      self.name, self.posActual, posDes, self.posError, self.posErrorD, self.posErrorI, velCmd, self.direction)


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
    def Main(self):
        rospy.spin()
                
                
                

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--name", type="str", dest="name", default='joint',
                        help="name=A|B|C|etc.")
    parser.add_option("--id", type="str", dest="id", default='A',
                        help="id=A|B|C|etc.")
    #parser.add_option("--mode", type="str", dest="mode", default='position',
    #                    help="mode=position|velocity")
    (options, args) = parser.parse_args()

    try:
        node = RosSimpleStep(name=options.name, id=options.id)
        node.Main()
    except:
        rospy.loginfo("SS (%s) rosSimpleStep - postinit", options.name)
        
    
    
    
    
    
    
    
    
