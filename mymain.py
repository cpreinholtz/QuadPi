#import serial
#from RPIO import PWM
#servo = PWM.Servo();
#ser= serial.Serial('/dev/ttyACM0',9600) #this is the arduiono
##import os
import time
import datetime
import smbus
import math
from Adafruit_PWM_Servo_Driver import PWM
from numpy import matrix
##from numpy import linalg
from LSM9DS0 import *
bus = smbus.SMBus(1)


##import pigpio
##pi = pigpio.pi()
##pi.set_mode( 19 , pigpio.INPUT )
##pi.set_mode( 20 , pigpio.INPUT )
##pi.set_mode( 21 , pigpio.INPUT )
##pi.set_mode( 26 , pigpio.INPUT )
##pi.set_mode( 16 , pigpio.INPUT )
##pi.set_mode( 13 , pigpio.INPUT )



try:
    import RPi.GPIO as GPIO
except RunTimeError:
    print("error1234") 
    
GPIO.setmode(GPIO.BCM)#  this uses gpio numbering system
GPIO.setup(19, GPIO.IN)#thro
GPIO.setup(20, GPIO.IN)#elev
GPIO.setup(21, GPIO.IN)#rudd
GPIO.setup(26, GPIO.IN)#aile
GPIO.setup(16, GPIO.IN)#gear
GPIO.setup(13, GPIO.IN)#aux

##
##GPIO.setup(4,GPIO.OUT)
##GPIO.setup(17,GPIO.OUT)
##GPIO.setup(18,GPIO.OUT)
##GPIO.setup(27,GPIO.OUT)
#-------------------------------------------- functions IMU

##def mypwmout(len , gpin):
##    len = len/throttlemult#convert into microseconds maybe need typecasting? 
##    if len>2200 or len<300:
##        print "improper pwm out len"
##        return -1
##    
##    else:
##        GPIO.output(gpin,1)#begin pulse
##        beginpulseout = datetime.datetime.now()
##        otime = datetime.datetime.now() - beginpulseout
###        otimeint = otime.microseconds
##        while otime.microseconds<len:
##            otime = datetime.datetime.now() - beginpulseout
###            otimeint = otime.microseconds
##        GPIO.output(gpin,0)#end pulse
###        print str(otime.microseconds) + " is actual len   "+str(len)+" was desired " 
##        return -1




##aadd = 0x04
##
##
##d0add = 0x10
##d1add = 0x12
##d2add = 0x14
##d3add = 0x16
##d4add = 0x18
##d5add = 0x1A
##
##def readA(ARDUINO_ADDR, DATA_ADDR):
##    bus.write_byte(ARDUINO_ADDR, DATA_ADDR)
##    l = bus.read_byte_data(A_ADDR)
##    bus.write_byte(ARDUINO_ADDR, DATA_ADDR+0x01)
##    l = bus.read_byte_data(A_ADDR)
##
##    combined = (l | h <<8)
##    return combined  if combined < 32768 else combined - 65536





def writeACC(register,value):
        bus.write_byte_data(ACC_ADDRESS , register, value)
        return -1

def writeMAG(register,value):
        bus.write_byte_data(MAG_ADDRESS, register, value)
        return -1

def writeGRY(register,value):
        bus.write_byte_data(GYR_ADDRESS, register, value)
        return -1


    

def readACCx():
    acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
    acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
    acc_combined = (acc_l | acc_h <<8)

    return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCy():
    acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
    acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
    acc_combined = (acc_l | acc_h <<8)

    return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCz():
        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
        acc_combined = (acc_l | acc_h <<8)
        return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readMAGx():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536


def readMAGy():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536


def readMAGz():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536



def readGYRx():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
  

def readGYRy():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

def readGYRz():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
    
def setServoPulse(channel, pulse):
    pulseLength = 1000000                   # 1,000,000 us per second
    pulseLength /= f                       # 60 Hz
    print "%d us per period" % pulseLength
    pulseLength /= 4096                     # 12 bits of resolution
    print "%d us per bit" % pulseLength
    pulse *= 1000
    pulse /= pulseLength
    #pwm.setPWM(channel, 0, pulse)
    return -1

#---noncritical initialisation----------------------------------------------------------noncritical initialisation
pulseb=datetime.datetime.now()
pulsebegin=pulseb
etime=pulseb
etimeint=0
start = datetime.datetime.now()
##LPavg = .032 
debounce1 = 0
debounce1t = 15
debounce2=0
debounce2t=300

xoffset= 0.0
yoffset= 0.0#for calibration
offsetmult = 0.5 #lpf for calibration
rgxoff = 0.0
rgyoff = 0.0
max = 0.0001

#initialise the accelerometer
writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
writeACC(CTRL_REG2_XM, 0b00100000) #+/- 16G full scale

#initialise the magnetometer
writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
writeMAG(CTRL_REG6_XM, 0b01100000) #+/-12gauss
writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

#initialise the gyroscope
writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
writeGRY(CTRL_REG4_G, 0b00110000) #Continuos update, 2000 dps full scale

acc1y = 0.0
acc2y = 0.0
acc3y = 0.0
acc1x = 0.0 
acc2x = 0.0
acc3x = 0.0


#---critical setup------------------------------------------- ---------------critical setup
f = 50#pwm freq maybe use 60
pwm = PWM(0x40)
pwm.setPWMFreq(60)  #maybe use 60? maybe use f
##setServoPulse(0,150)
Tonestep = 1000000*1/(f*4096.0)# each step in microsecons
##print "Tonestep"
##print Tonestep



thresh1 = 35000# timeout reciever
EnableFixedLP = 1 #  0 =go as fast as possible  1,2,3,or4 = fixed loop
LPmicthresh = 55000 #this is how long each loop will be if enbled above


RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
LP = (LPmicthresh+240)/1000000.0   	# Loop period = 35ms.   This needs to match the time it takes each loop to run


loopcnt=0
loopmax=6# controls how often we read the controller Higher is less frequent 5 will get all channels
loopmax1 = loopmax-1
nch = 6 # number of channels gathered from reciever
lc1 = 0
lc2 = loopmax/nch
lc3 = 2*loopmax/nch
lc4 = 3*loopmax/nch
lc5 = (4*loopmax/nch)
lc6 = 5*loopmax/nch
##print lc1
##print lc2
##print lc3
##print lc4
##print lc5
##print lc6



##fp = open('p','r')
##kp = float(fp.readline())
##fp.close()
##fi = open('i','r')
##ki = float(fi.readline())
##fi.close()
##fd = open('d','r')
##kd = float(fd.readline())
##fd.close()

err1 = False

with open('pid','r') as fil:
    with open('pid2', 'r') as fil2:
        try:#try to get kp ki and kd
            kp = float(fil.readline())
            ki = float(fil.readline())
            kd = float(fil.readline())
            
        except:# try to load data from the backup file
            try:
                kp = float(fil2.readline())
                ki = float(fil2.readline())
                kd = float(fil2.readline())
                
            except:#use default values if both files are corrupted
                kp = 6.0
                ki = 0.001
                kd = 0.001
                err1 = True
    

throttlemult=1.0# servoblaster ises increments of 10us, = .1
kp = kp*throttlemult # PID control values
ki = ki*throttlemult
kd = kd*throttlemult
start_integrator = 5.0 #digital INT only activates with small errors
ierrormax = 100.0
ky = 0.5*throttlemult#yaw gain

dmin = 900.0
dmax = 2100.0
maxrollangle=10.0 #when 2000 or 1000 pwm
pwmcenter = 1467.0#used to find 0 in roll yaw elev
pwmmax=650#per side
pwmmult = -(maxrollangle)/pwmmax #negative because reciver is opposite sensor

dwindow = 500.0
activate = 1150.0
inactive = 1000.0
active=False
zerodeg = 5.0
oldd0 = .3
zeropwm = 50.0
desired= [inactive,0.0,0.0,0.0,1900.0,1000.0] # throttle,  pitch, roll, yaw, aux    length must match numchannels
dlastmult=.5
desirednext= [900.0,0.0,0.0,0.0,1900.0,1000.0]
r0err = 0
r0failsafe = 100




error= [0.0,0.0] #pitch roll
errorlast= [0.0,0.0];
derror= [0.0,0.0]
ierror= [0.0,0.0]
sensor = [0.0,0.0]# ####### pitch roll x y mag?

yawmultiplier = 5.0# #################maybey use magnetometer
yawtrim=0.0


#---kalman filter initialization---------------------- -------------------kalman filter initialization
#kalman filter initialization
AA =  0.80      # Complementary filter constant
cfx = 0.0
cfy = 0.0
cfxlast = 0.0
cfylast = 0.0



theta0 = 0.0
thetadot0 = 0.0
thetadotbias = 0.0

q1 = 20.0
q2 = 450.0  #  500 300
q3 = .5

p1 = 1000.0
p2 = p1
p3 = p1

r1 = 200.0
r2 = 750.0

s1 = 100.0
s2 = 100.0



Xx = matrix( [[theta0], [thetadot0], [thetadotbias] ])#the current state of the x direction

Xy = matrix( [[theta0], [thetadot0], [thetadotbias] ])#the current state of the y direction

F = matrix([ [1.0,LP,-LP], [0.0,1.0,0.0], [0.0,0.0,1.0] ])

Q = matrix([  [q1,0.0,0.0], [0.0,q2,0.0], [0.0,0.0,q3] ])



Px =  matrix([  [p1,0.0,0.0], [0.0,p2,0.0], [0.0,0.0,p3] ])
Py =  matrix([  [p1,0.0,0.0], [0.0,p2,0.0], [0.0,0.0,p3] ])

Yx = matrix( [[theta0], [thetadot0] ])
Yy = matrix( [[theta0], [thetadot0] ])

Zx = matrix( [[theta0], [thetadot0] ])
Zy = matrix( [[theta0], [thetadot0] ])

H = matrix([  [1.0,0.0,0.0], [0.0,1.0,0.0] ])

R = matrix([  [r1,0.0], [0.0,r2] ])

Sx = matrix([  [s1,0.0], [0.0,s2] ])
Sy = matrix([  [s1,0.0], [0.0,s2] ])

Kx = matrix([  [0.0,0.0], [0.0,0.0] ])
Ky = matrix([  [0.0,0.0], [0.0,0.0] ])

ID = matrix([  [1.0,0.0,0.0], [0.0,1.0,0.0], [0.0,0.0,1.0] ])




go = True
normal =  True  #calibration mode =0 #  desired 0 cutoff
normal2 = True #cal mode 0 # error
normal3 = True # cal mode 0 # single channel
calch = 0 # calibration channel
pidtune=True # 1 makes the swith do pid           0 is hover switch
start = datetime.datetime.now()

while go:
 
##---sensor-----------------------------------------------------sensor
    # ##############################get sensor values
    ACCx = readACCx()
    ACCy = readACCy()
    ACCz = readACCz()
    GYRx = readGYRx()
    GYRy = readGYRy()

    
    #Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG
    AccXangle -= 180.0
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0 
        
    acc3x = acc2x
    acc2x= acc1x
    acc1x = AccXangle
    
    acc3y = acc2y
    acc2y = acc1y
    acc1y = AccYangle
    
    accMAx = (acc1x+acc2x+acc3x)/3.0
    accMAy = (acc1y+acc2y+acc3y)/3.0
        
	#Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN  
    
##    if desired[5]> 1450.0: #calibration
##        rgxoff = offsetmult*rgxoff+(1-offsetmult)*rate_gyr_x
##        rgyoff = offsetmult*rgyoff+(1-offsetmult)*rate_gyr_y
##    else:
##        rate_gyr_x = rate_gyr_x-rgxoff
##        rate_gyr_y = rate_gyr_y-rgyoff
     
##---CFilter---------------------------------------------------------Cfilter

    cfx =AA*(cfx+rate_gyr_x*LP) +(1 - AA) * accMAx
    cfy =AA*(cfy+rate_gyr_y*LP) +(1 - AA) * accMAy
    cfrx = (cfx - cfxlast)/LP
    cfry = (cfy - cfylast)/LP
    cfxlast = cfx
    cfylast = cfy
    
##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after sensor in miliSeconds" ###debug only 
    
##---kalmanx---------------------------------------------------------kalmanx
##    rate_gyr_x   AccXangle

    Xx = F*Xx  #make a guess of current state ased on last state
    
    Px = F*Px*F.T +Q   #guess covariance
    
##    Zx[0,0] =  cfx #get new sensor values
##    Zx[1,0] =  rate_gyr_x
    Zx[0,0] =  cfx #get new sensor values
    Zx[1,0] =  cfrx
    
    Yx = Zx - H*Xx # find difference between sensors and model
    
    Sx = H*Px*H.T + R #get covariance 
    
    Kx = Px*H.T*Sx.I   #kalman gain
    
    Xx = Xx + Kx*Yx  #adjust guess based on kalman gain and sensor readings
    
    Px = (ID-Kx*H)*Px #fix covariance
    
##---kalmany---------------------------------------------------------kalmany
##    rate_gyr_x   AccXangle

    Xy = F*Xy  #make a guess of current state ased on last state
    
    Py = F*Py*F.T +Q   #guess covariance
    
##    Zy[0,0] =  cfy #get new sensor values
##    Zy[1,0] =  rate_gyr_y
    Zy[0,0] =  cfy #get new sensor values
    Zy[1,0] =  cfry
    
    Yy = Zy - H*Xy # find difference between sensors and model
    
    Sy = H*Py*H.T + R #get covariance 
    
    Ky = Py*H.T*Sx.I   #kalman gain
    
    Xy = Xy + Ky*Yy  #adjust guess based on kalman gain and sensor readings
    
    Py = (ID-Ky*H)*Py #fix covariance
    
    
    #####calibration
    if desired[5]>1450.0:  #calibration
        sensor[1] = Xy[0,0]
        sensor[0] = Xx[0,0]
        xoffset = offsetmult*xoffset+(1-offsetmult)*sensor[0]
        yoffset = offsetmult*yoffset+(1-offsetmult)*sensor[1]
        
    else: #caliration complete
        sensor[1] = Xy[0,0]-yoffset
        sensor[0] = Xx[0,0]-xoffset

##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after kalman  in miliSeconds" ###debug only 
    
    
##---error---------------------------------------------------------error
    if  pidtune: #locked at 0
        error[0] = 0.0-sensor[0]
        error[1] = 0.0 -sensor[1]
    else:
        error[0] = desired[1]-sensor[0]
        error[1] = desired[2] -sensor[1]
    derror[0] = error[0]-errorlast[0]
    derror[1] = error[1]-errorlast[1]
    if active:
        if abs(error[0])<start_integrator and abs(ierror[0])<ierrormax:        
            ierror[0]+=error[0]
        if abs(error[1]) <start_integrator and abs(ierror[1])<ierrormax:
            ierror[1]+=error[1]
    errorlast[0] = error[0]
    errorlast[1] = error[1]
    
##---PID-------------------------------------------------------------PID
    base = desired[0]*throttlemult# ###maybe adust? need a 1000 to 1900 spread
    throt = [base,base,base,base] # FL FR BL BR
    
    if active and normal2:

        # -------------------------------pitch
        throt[0] +=(kp*error[0] +kd*derror[0] + ki*ierror[0])
        throt[1] +=(kp*error[0] +kd*derror[0] + ki*ierror[0])
        throt[2] -=(kp*error[0] +kd*derror[0] + ki*ierror[0])
        throt[3] -=(kp*error[0] +kd*derror[0] + ki*ierror[0])
        # -------------------------------roll
        throt[0] +=(kp*error[1] +kd*derror[1] + ki*ierror[1])
        throt[1] -=(kp*error[1] +kd*derror[1] + ki*ierror[1])
        throt[2] +=(kp*error[1] +kd*derror[1] + ki*ierror[1])
        throt[3] -=(kp*error[1] +kd*derror[1] + ki*ierror[1]) 
        # -------------------------------yaw
    #    throt[0] +=yawtrim
        #throt[1] -=yawtrim
        #throt[2] -=yawtrim
        #throt[3] +=yawtrim
        if not pidtune:
            throt[0] +=(desired[3]*ky)
            throt[1] -=(desired[3]*ky)
            throt[2] -=(desired[3]*ky)
            throt[3] +=(desired[3]*ky)






##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after pid in miliSeconds" ###debug only
    
####---pwm------------------------------------------------------------PWM

#servo blaster
##    os.system("echo 0="+str( int(throt[0]))+ "> /dev/servoblaster")  # FL gpio4
##    os.system("echo 1="+str( int(throt[1]))+ "> /dev/servoblaster")  #FRgpio17
##    os.system("echo 2="+str( int(throt[2]))+ "> /dev/servoblaster")   #BL gpio18
##    os.system("echo 3="+str( int(throt[3]))+ "> /dev/servoblaster")   #BR gpio27
##    

#my function using gpio
##    mypwmout(throt[0] , 4)
##    mypwmout(throt[1] , 17)
##    mypwmout(throt[2] , 18)
##    mypwmout(throt[3] , 27)
    
##    print "throt0: %5i 1: %5i  2: %5i  3: %5i" %(int(throt[0]), int(throt[1]), int(throt[2]) , int(throt[3]))###debug only
    
    
    throt[0] = int( throt[0] / Tonestep)
    throt[1] = int( throt[1] / Tonestep)
    throt[2] = int( throt[2] / Tonestep)
    throt[3] = int( throt[3] / Tonestep)
    
#adafruit
    if normal3:##calibraton mode
            pwm.setPWM(0 , 0 , throt[0])
            pwm.setPWM(1 , 0 , throt[1])   
            pwm.setPWM(2 , 0 , throt[2])
            pwm.setPWM(3 , 0 , throt[3])
    else:
        if calch == 0:
            pwm.setPWM(0 , 0 , throt[0])
        elif calch == 1:
            pwm.setPWM(1 , 0 , throt[1])
        elif calch == 2   :     
            pwm.setPWM(2 , 0 , throt[2])
        elif calch == 3:
            pwm.setPWM(3 , 0 , throt[3])

    
    
##    print "throt0: %5i 1: %5i  2: %5i  3: %5i" %(int(throt[0]), int(throt[1]), int(throt[2]) , int(throt[3]))##debug only




##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after output in miliSeconds" ###debug only




##---desired0throt---------------------------------------------------------desired
    if loopcnt==lc1:# throttle,  pitch, roll, yaw, gear, aux 
        thisgpio = 19
        thisdesired = 0
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
            if etimeint >=thresh1:
                tru=0
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
                tru=0
                
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
            
        desirednext[thisdesired]= float(etime.microseconds)
        if desirednext[thisdesired] <dmin or desirednext[thisdesired]>dmax:
            tru=0
        if tru:
            r0err=r0err-1;
            desired[thisdesired] = oldd0*desired[thisdesired]+(1-oldd0)*desirednext[thisdesired]
            if desired[thisdesired]<activate and normal:
                desired[thisdesired]=inactive
                active = False
            else:
                active=True
               
        else:#error
            r0err=r0err+1;
            
        if r0err >r0failsafe:#reciver timeout
            desired[thisdesired] = inactive
            if tru==0:
                r0err = r0failsafe+10
##        else:
##            print " reciever error throt" ###debug only     
            
##---desired5aux---------------------------------------------------------desired
    elif loopcnt==lc6:
        thisgpio=13
        thisdesired=5#used to determine if calibration mode is on
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)

            if etimeint >=thresh1:
                tru=0
##                print " 1 reciever error aux" ###debug only     
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
##                print etimeint ###debug only     
##                print " 2 reciever error aux" ###debug only     
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
##                print " 3 reciever error aux" ###debug only     
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
        desirednext[thisdesired]= float(etime.microseconds)
        if desirednext[thisdesired] <dmin or desirednext[thisdesired]>dmax:
            tru=0
        if tru:
            desired[thisdesired] = desired[thisdesired]*dlastmult+desirednext[thisdesired]*(1-dlastmult)
##        else:
##            print " reciever error aux" ###debug only     





        
        
        
        
        
            
##---desired4gear---------------------------------------------------------desired
    elif loopcnt==lc2:
        thisgpio=16
        thisdesired=4#used to determine if gear is locked
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)

            if etimeint >=thresh1:
                tru=0
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
            #etimeint = int(etime.microseconds)
        desirednext[thisdesired]= float(etime.microseconds)
        if desirednext[thisdesired] <dmin or desirednext[thisdesired]>dmax:
            tru=0
        if tru:
            desired[thisdesired] = desired[thisdesired]*dlastmult+desirednext[thisdesired]*(1-dlastmult)
##        else:
##            print " reciever error gear" ###debug only     



        
    elif desired[4]>1500.0 and not pidtune: #locked

        desired[1] = 0.0
        desired[2] = 0.0
        desired[3] = 0.0
        
        
        
        
        
##---desired3yaw---------------------------------------------------------desired
    elif loopcnt==lc3:# throttle,  pitch, roll, yaw, aux 
        thisgpio = 21
        thisdesired=3
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
            if etimeint >=thresh1:
                tru=0
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
        desirednext[thisdesired]= float(etime.microseconds)-pwmcenter
        
        if desirednext[thisdesired] <-pwmmax or desirednext[thisdesired]>pwmmax:
            tru=0
        if tru:
            desired[thisdesired] = desired[thisdesired]*dlastmult+desirednext[thisdesired]*(1-dlastmult)
            if desired[thisdesired] >-zeropwm and desired<zeropwm:#####maybet change this might want to trim yaw
                desired[thisdesired] = 0.0

##        else:
##            print " reciever error yaw" ###debug only         
            
        
        
        
        
##---desired1pitch---------------------------------------------------------desired
    elif loopcnt==lc4:
        thisgpio = 20
        thisdesired=1
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
            if etimeint >=thresh1:
                tru=0
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
        desirednext[thisdesired]= (float(etime.microseconds)-pwmcenter)*pwmmult
        
        if desirednext[thisdesired] <-maxrollangle or desirednext[thisdesired]>maxrollangle:
            tru=0
        if tru:
            desired[thisdesired] = desired[thisdesired]*dlastmult+desirednext[thisdesired]*(1-dlastmult)
            #if desired[thisdesired] >-zerodeg and desired<zerodeg:
                #desired[thisdesired] = 0.0

##        else:
##            print " reciever error pitch" ###debug only     
    
    
##---desired2roll---------------------------------------------------------desired
    elif loopcnt==lc5:
        thisgpio= 26
        thisdesired=2
        
        tru=1
        val = GPIO.input(thisgpio)
        pulsebegin=datetime.datetime.now()
        while val ==1:
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
            #pulseb=datetime.datetime.now()
            #elapsedtime = pulseb-pulsebegin
            if etimeint >=thresh1:
                tru=0
                break;
        while val==0:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsebegin
            etimeint = int(etime.microseconds)
        pulsea=datetime.datetime.now()##last pulsea read (rising EDGE)
        while val ==1:
            if etimeint >=thresh1:
                tru=0
                break;
            val = GPIO.input(thisgpio)
            etime = datetime.datetime.now() -pulsea
            #etimeint = int(etime.microseconds)
        desirednext[thisdesired]= (float(etime.microseconds)-pwmcenter)*pwmmult
        
        if desirednext[thisdesired] <-maxrollangle or desirednext[thisdesired]>maxrollangle:
            tru=0
        if tru:
            #avg = .98*avg+.02*float(etime.microseconds)
            desired[thisdesired] = desired[thisdesired]*dlastmult+desirednext[thisdesired]*(1-dlastmult)
            #if desired[thisdesired] >-zerodeg and desired<zerodeg:
            #    desired[thisdesired] = 0.0

##        else:
##            print " reciever error roll" ###debug only     





##
##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after desired in miliSeconds" ###debug only 
    
    ############pid tuning  
    if pidtune:
        if desired[4]>1450.0:#pid tuning mode
            if desired[1]<-maxrollangle/3.0:
                kp = kp+.05#p
            elif desired[1] >maxrollangle/3.0:
                kp = kp-.2#p
                if kp<0.05:
                    kp = 0.0
            if desired[2]>maxrollangle/3.0:
                ki = ki+.001#i
            elif desired[2] <-maxrollangle/3.0:
                ki = ki-.001#i
                if ki<0.0:
                    ki = 0.0
            if desired[3]<-300.0:
                kd = kd+.001#d
            elif desired[3] >300.0:
                kd = kd-.001#d
                if kd<0.0:
                    kd = 0.0
        done = False#save pid
        while not done:
            try:
                with open('pid','w') as fil:
                    fil.write(str(kp)+"\n"+ str(ki)+"\n"+str(kd)+"\n")
                done = True
            except KeyboardInterrupt:
                done=False
                
        done = False#save a backup pid
        while not done:
            try:
                with open('pid2','w') as fil2:
                    fil2.write(str(kp)+"\n"+ str(ki)+"\n"+str(kd)+"\n")
                done = True
            except KeyboardInterrupt:
                done=False
                

















    
    if loopcnt<loopmax1:
        loopcnt= loopcnt+1
    else:
        loopcnt=0
        
##---wait---------------------------------------------------------wait

    if EnableFixedLP:
        LPEND = datetime.datetime.now()-start ####debug only or tie to LP
##        LPmic = LPEND.microseconds
        LPsecondsleft =  float(LPmicthresh - LPEND.microseconds)/1000000.0 #
        if LPsecondsleft>.00001:# need a positive number
            time.sleep(LPsecondsleft)
            
            
        
##        print str(LPmic) +" is final calc loop time in miliSeconds" ####debug only 
##        while LPmic<LPmicthresh:
##            LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##            LPmic = LPEND.microseconds
##            if debounce2<debounce2t:
##                debounce2+=1
##            else:
##                print str(LPmic) +" is current loop time in miliSeconds" ###debug only 
##                debounce2=0


        LPEND = datetime.datetime.now()-start ####debug only or tie to LP
        LPmic = LPEND.microseconds
        print str(LPmic) +" is total loop time in miliSeconds" ###debug only 
        start = datetime.datetime.now()
        
        
##---done---------------------------------------------------------done
        
        
        
        
        
    else :###debug only
        LPEND = datetime.datetime.now()-start
        LP = float(LPEND.microseconds)/1000000
        LPavg = .9*(LPavg)+LP*.1
##        print str(LPavg) +" is average loop time in Seconds" ###debug only 
        print str(LP) +" is loop time in Seconds" ###debug only 
        start = datetime.datetime.now()###debug only or tie to LP
        if LP>max:
            debounce1+=1
            if debounce1>debounce1t:
                max = LP
##        print str(max) +" is max loop time in Seconds" ####debug only
    
    
##    print "throt0: %5i 1: %5i  2: %5i  3: %5i" %(int(throt[0]), int(throt[1]), int(throt[2]) , int(throt[3]))
    

    # #################printer debugg
    #print "FL: %3s  FR:  %3s   BL: %3s  BR: %3s" %(str(int(throt[0])), str(int(throt[1])), str(int(throt[2])), str(int(throt[3])))
##    if active ==1:
##        print "filterx: %5.3f   accxlp  %5.3f   gyrox %5.3f    " %(sensor[0],AccXLP,deltaXgyro)
    print "sensor x %3.5f      sensory %3.5f      "%(sensor[0], sensor[1] )
    print "cfxr %3.5f   rgyrx %3.5f   y1 %3.5f      y2 %3.5f       "%(cfrx, rate_gyr_x , cfry , rate_gyr_y )
    print "accmax %3.5f      y %3.5f      "%(accMAx, accMAy )
    #print "kp %3.8f      ki %3.8f     kd %3.8f       "%(kp,ki,kd)
    
##        print "filterx: %5.3f   accxlp  %5.3f   gyrox %5.3f    " %(sensor[0],AccXLP,deltaXgyro)
    

    
##    LP = float(LPEND.microseconds)/1000000
##    LPavg = .9*(LPavg)+LP*.1
##    print str(LPavg) +" is average loop time in Seconds" ####debug only 
##    print str(LP) +" is loop time in Seconds" ####debug only 

##    print "e: %3.5f  de:  %3.5f   ie: %3.5f" %(error[0], derror[0], ierror[0] )
##    print "e: %3.5f  de:  %3.5f   ie: %3.5f" %(error[1], derror[1], ierror[1] )


        
        
  #  print "throt: %5.2f   pitch: %5.2f   roll: %5.2f   yaw: %5.2f   lock: %5.2f" %(desired[0],desired[1], desired[2] ,desired[3], desired[4])
    #print "avg: %5.2f" %(avg)
    
    
