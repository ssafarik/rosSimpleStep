import numpy as np
import time
import simple_step
import atexit
import pylab
import logging

# initialize stepper motors and usb drivers

class StepperMotor:

    def __init__(self,sernum='0.0.A', gr=1, clkdir_mult=1, ind_per_rev=12800, vel_max=30000, vel_min=0):
    
        # initialize log file
        #LOG_FILENAME = 'motor_log_%d.out' % (motorID)
        #logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)
            
        print('Initializing Motor.... sernum: ', sernum, 'gear ratio: ', gr)
        self.sernum = sernum
        self.gr = gr
               
        # initialize USB controller
        self.dev = simple_step.Simple_Step(serial_number=sernum)
        self.dev.set_zero_pos(self.dev.get_pos())
        self.dev.set_vel_setpt(0,io_update=True)
        self.dev.set_mode(simple_step.VELOCITY_MODE)  
        self.mode = self.dev.get_mode()
        self.dev.start()
        
        # motor constants
        self.clkdir_mult = clkdir_mult
        self.ind_per_rad = ind_per_rev*self.gr/(2.0*np.pi*self.clkdir_mult)
        self.rad_per_ind = 1.0/self.ind_per_rad
        self.zeropos = self.dev.get_pos()*self.rad_per_ind
        self.centerpos = 0
        self.limithi = 1
        self.limitlo = -1
        self.hitlimit = 0
        self.vel_max = vel_max
        self.vel_min = vel_min
        
        # plot constants
        self.tstart = 0
        
        atexit.register(self.atexit_func)
        
    # START / STOP
    def start(self):
        if self.dev.get_status() is not 'running':
            self.dev.start()
    def stop(self):
        self.dev.stop()
        
    # Basic GET funcs
    def getstatus(self):
        print self.dev.get_status()
    def getzero(self):
        return self.zeropos
    def getlimits(self):
        return [self.limitlo, self.limithi]
    def getsoftlimits(self):
        return [self.limitlo+limitbuffer, self.limithi-limitbuffer]
    def getmode(self):
        return self.mode
    def getint(self):    
        return self.dev.get_ext_int()
        
    def getpos(self, units='radians'):
        pos = self.dev.get_pos()  
        if units is 'radians':
            pos = pos*self.rad_per_ind
        return pos
        
    def getvel(self, units='radians'):
        vel = self.dev.get_vel()
        direc = self.dev.get_dir()
        if direc is 'positive':
            signed_vel = vel
        elif direc is 'negative': 
            signed_vel = vel*(-1.0) 
        if units is 'radians':
            signed_vel = signed_vel*self.rad_per_ind
        return signed_vel
        
    # Basic SET funcs
    def setmode(self,motormode):
        if motormode is 'velocity':
            self.dev.set_mode(simple_step.VELOCITY_MODE)
        if motormode is 'position':
            self.dev.set_mode(simple_step.POSITION_MODE)
        self.mode = self.dev.get_mode()
        
    def gotocenter(self):
        self.setpos(self.centerpos)
        
    def setint(self, intmode):    
        if type(intmode) == str:
            self.dev.set_ext_int(intmode)
        else:
            if intmode == 0:
                self.dev.set_ext_int('disabled')
            elif intmode == 1:
                self.dev.set_ext_int('enabled')
        return self.dev.get_ext_int()
   
    def setvel(self, veldes, change_direction = '1', units='radians'):
        if units is 'radians':
            veldes = veldes*self.ind_per_rad
        
        if self.mode is 'position':
            self.dev.set_vel_setpt(0,io_update=True)
            self.setmode('velocity')
       
        if veldes > 0:
            direction = simple_step.POSITIVE
        else:
            direction = simple_step.NEGATIVE
            
        # Set limits on controlled velocity 
        if np.abs(veldes) > self.vel_max:
            veldes = self.vel_max*np.sign(veldes)
        elif np.abs(veldes) < self.vel_min:
            veldes = self.vel_min*np.sign(veldes)
    
        change_direction=1
        if change_direction:
            self.dev.set_dir_setpt(direction,io_update=False)
        self.dev.set_vel_setpt(np.abs(veldes),io_update=True)
        
    def setpos(self, posdes, units='radians', style='direct'):
        if units is 'radians':
            posdes = posdes*self.ind_per_rad
        if self.mode is 'velocity':
            self.setmode('position')

        if style is 'soft':    
            self.dev.soft_ramp_to_pos(posdes,10,500)
        elif style is 'direct':
            self.dev.set_pos_setpt(posdes)
                
    # FIND ZEROS
    def findzeros(self):
        logging.debug('log: finding zeros')
    
        speed = .4
        sleeptime = 1

        self.dev.set_zero_pos(self.dev.get_pos())

        # Drive Motor A, round 1
        self.setint(1)
        self.setvel(speed)
        time.sleep(0.05)
        tmp_vel = self.getvel()
        logging.debug('%f: round 1 velocity %f' % (time.time(), tmp_vel))
          
        # Loop until the motor stops
        while np.abs(self.getvel())>0:
            pass
        posA_1 = self.getpos(units='indices')      

        # Drive Motor A, round 2
        self.setint(0)       
        self.setvel(-1.0*speed)
        self.start()
        time.sleep(0.05)
        tmp_vel = self.getvel()
        logging.debug('%f: round 2 velocity %f' % (time.time(), tmp_vel))
        time.sleep(0.5)
        self.setint(1)
        tmp_vel = self.getvel()
        logging.debug('%f: round 2 velocity %f' % (time.time(), tmp_vel))
        
        # Loop until the motor stops
        while np.abs(self.getvel())>0:
            pass
        posA_2 = self.getpos(units='indices')
   
        # zero value for A        (pos 2 is less then pos 1)
        self.centerpos = (((posA_1-posA_2)/2)+posA_2)*self.rad_per_ind  
        self.limitlo = -1.0*(posA_1-posA_2)/2.0*self.rad_per_ind
        self.limithi = (posA_1-posA_2)/2.0*self.rad_per_ind 
         
        # go to center pos
        self.setint(0)
        self.setpos(self.centerpos)
        self.start()
        logging.debug('%f: moving to final pos' % (time.time()))
        tmp_vel = self.getvel()
        logging.debug('%f: moving to final pos %f' % (time.time(), tmp_vel))
        time.sleep(0.5)
        self.setint(1)
        tmp_vel = self.getvel()
        logging.debug('%f: moving to final pos %f' % (time.time(), tmp_vel))
        
        while np.abs(self.getvel())>0:
            pass
        self.dev.set_zero_pos(int(round(self.centerpos*self.ind_per_rad)))

        print('posA1*: ', posA_1, 'posA2: ', posA_2, 'center pos: ', self.centerpos)
        print('lo limit set: ', self.limitlo, '  high limit set: ', self.limithi)       
        print('usb zero: ', self.getpos())
        print(self.getint())
        return self.limitlo, self.limithi
        
    def infinity(self, duration=2, speed=-1):
        print 'infinity run'
        self.setvel(speed)
        time.sleep(duration)
        self.setvel(0)
        self.dev.set_zero_pos(int(round(self.getpos('indices'))))       
        
    def zerostart(self):
        print 'zero start'
        self.setvel(0)
        self.dev.set_zero_pos(int(round(self.getpos('indices'))))
        
    # DEBUGING
    def sintest(self, AMP=0.5, FREQ=1, TIME=5, mode='velocity'):
    
        # freq in hertz
        # amp in radians
        
        self.tstart = time.time()
        t = time.time()-self.tstart
        self.setpos(0)
        pylab.ion()
        
        print('**Running Sin Test**')
        
        while t<TIME:
            t = time.time()-self.tstart
            ctrl = AMP*np.sin(FREQ*2.0*np.pi*t)
            
            if mode == 'velocity':
                self.setvel(ctrl)
                output = self.getvel()
            if mode == 'pos':
                self.setpos(ctrl)
                output = self.getpos()
            
            pylab.plot([t],[output],color='blue', marker='*')
            pylab.plot([t],[ctrl],color='red')
            pylab.draw()
        self.stop()
   
    # QUITTING
    def kill(self):
        self.setvel(0)
        self.stop()
        self.dev.close()     
        print('goodbye! you killed me!')

    def atexit_func(self):
        # Close device        
        if 0: 
            self.dev.close()
            del self.dev
            dev2 = simple_step.Simple_Step(self.sernum)
            dev2.set_vel_setpt(0)
            dev2.stop()
            print 'device', sernum, ' stopped'
            print('goodbye! killed on exit')
        
        
    def cleanup(self):
        print '*'*100
        print 'test'
        self.setvel(0)
        print '1'
        self.stop()
        print 'device', self.sernum, ' stopped'
        self.dev.close()
        print 'device', self.sernum, ' closed'
        
        
