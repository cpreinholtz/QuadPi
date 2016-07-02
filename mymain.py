
import time
import datetime
import smbus
import math
from LSM9DS0 import *
from Adafruit_PWM_Servo_Driver import PWM
from numpy import matrix
bus = smbus.SMBus(1)
try:
    import RPi.GPIO as GPIO
except RunTimeError:
    print("error1234") 

myled = 26    
GPIO.setmode(GPIO.BCM)#  this uses gpio numbering system
GPIO.setup(myled, GPIO.OUT)#led

GPIO.output(myled, 1)
time.sleep(.3)
GPIO.output(myled, 0)
time.sleep(.3)
GPIO.output(myled, 1)
time.sleep(.3)
GPIO.output(myled, 0)
time.sleep(.3)

import cwiid
print 'Press 1 + 2 on your Wii Remote now ...'
try:
  wii=cwiid.Wiimote()
except RuntimeError:
  print "Error opening wiimote connection"
  quit()
wii.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_NUNCHUK | cwiid.RPT_STATUS
time.sleep(2)
s1 = wii.state
b = int(s1['battery']/10)
if b>15:b=15
if b<1:b=1
wii.led = b


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

def savepid(kp,ki,kd):
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

#---noncritical initialisation----------------------------------------------------------noncritical initialisation



##LPavg = .032 

##debounce1 = 0
##debounce1t = 15
##debounce2=0
##debounce2t=300

xoffset= 0.0
yoffset= 0.0#for calibration
offsetmult = 0.5 #lpf for calibration
rgxoff = 0.0
rgyoff = 0.0
rgxn = 0.0
rgyn = rgxn
##max = 0.0001

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

wcal = False
wpitch = 0.0
wroll = 0.0
dth = 1000.0#inactive
dint=20.0#changes how much hitting throttle does per cycle
loopcnt=0
loopmax=300

#---critical setup------------------------------------------- ---------------critical setup

c1=False
cxavg = 10.0 #this is only used for led indicator light
cyavg = 10.0
clp = 0.9
cth = 0.6

f = 50#pwm freq maybe use 60
pwm = PWM(0x40)
pwm.setPWMFreq(60)  #maybe use 60? maybe use f
##setServoPulse(0,150)
Tonestep = 1000000*1/(f*4096.0)# each step in microsecons

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
LP = .0175 	# Loop period = 30ms.   This needs to match the time it takes each loop to run

start_integrator = 5.0 #digital INT only activates with small errors
ierrormax = 100.0
##ky = 0.5#yaw gain

maxrollangle=5.0 #when 2000 or 1000 pwm


activate = 1050.0
inactive = 1000.0
thmax = 2000.0
active=False
base = inactive

error= [0.0,0.0] #pitch roll
errorlast= [0.0,0.0];
derror= [0.0,0.0]
ierror= [0.0,0.0]
sensor = [0.0,0.0]# ####### pitch roll x y mag?

# #################pid init
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
#---kalman filter initialization---------------------- -------------------kalman filter initialization
#kalman filter initialization
#AA =  0.87      # Complementary filter constant
AA = .7
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
normal =  True  #esc calibration mode =0 #  desired 0 cutoff
normal2 = True #esc cal mode 0 # error
normal3 = True #esc  cal mode 0 # single channel
calch = 0 #esc calibration channel
pidtune = True
EnableFixedLP = True #  0 =go as fast as possible  1,2,3,or4 = fixed loop
nun = False
if not nun:
    wii.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_STATUS
    


while go:
    start3 = datetime.datetime.now()
    start2 = time.time()
    start = time.clock()
    
    #---desired--------------------------------------------------------desired

    #no nunchuck
    s1 = wii.state
    buttons = s1['buttons']
    if dth<thmax and bool(buttons &cwiid.BTN_RIGHT):#th up
        dth = dth+dint;
    elif dth>inactive and bool(buttons & cwiid.BTN_LEFT):
        dth = dth-dint;
    if bool(buttons & cwiid.BTN_B) or (dth < activate):
        active = False
    elif(dth>activate) :
        active = True
        
    wyaw= 0.0
    #   pitch
    if bool(buttons & cwiid.BTN_1):#back
        wpitch =  maxrollangle 
    elif bool(buttons & cwiid.BTN_2):#forward
        wpitch = -maxrollangle
    else:
        wpitch = 0.0
    #roll
    if bool(buttons & cwiid.BTN_DOWN): #right
        wroll = - maxrollangle 
    elif bool(buttons & cwiid.BTN_UP):#left
        wroll = maxrollangle
    else:
        wroll = 0;
    whover = bool(buttons & cwiid.BTN_PLUS)
    wnhover = bool(buttons & cwiid.BTN_MINUS)
    wcal = bool(buttons & cwiid.BTN_A)
    
       
    if whover:
        wyaw = 0
        wroll = 0
        wpitch = 0
        if pidtune:
            if bool(buttons & cwiid.BTN_UP): 
                c1 = not c1
                GPIO.output(myled,c1)
                kp = kp+.05#p
            if bool(buttons & cwiid.BTN_RIGHT): 
                c1 = not c1
                GPIO.output(myled,c1)
                kp = ki+.001#i
            if bool(buttons & cwiid.BTN_LEFT): 
                c1 = not c1
                GPIO.output(myled,c1)
                kp = kd+.001#d 
            savepid(kp,ki,kd)
            
    elif wnhover:
        wyaw = 0
        wroll = 0
        wpitch = 0
        if pidtune:
            if bool(buttons & cwiid.BTN_UP): 
                c1 = not c1
                GPIO.output(myled,c1)
                kp = kp-.05#p
                if kp<0.05:
                    kp = 0.0
            if bool(buttons & cwiid.BTN_RIGHT): 
                c1 = not c1
                GPIO.output(myled,c1)
                ki = ki-.001#i
                if ki<0.001:
                    ki = 0.0
            if bool(buttons & cwiid.BTN_DOWN): 
                c1 = not c1
                GPIO.output(myled,c1)
                kd = kd-.001#d
                if kd<0.001:
                    kd = 0.0
            savepid(kp,ki,kd)
    
    
    
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
    
    accMAx = (acc1x+acc2x+acc3x)/3.0####average every 3 loops, consider averaging 3 in one loop
    accMAy = (acc1y+acc2y+acc3y)/3.0
        
	#Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN  
    
    if wcal: ###########calibration1
        rgxoff = offsetmult*rgxoff+(1-offsetmult)*rate_gyr_x
        rgyoff = offsetmult*rgyoff+(1-offsetmult)*rate_gyr_y
    else:
        rgxn = rate_gyr_x-rgxoff
        rgyn = rate_gyr_y-rgyoff
     
##---CFilter---------------------------------------------------------Cfilter

    cfx =AA*(cfx+rgxn*LP) +(1 - AA) * accMAx
    cfy =AA*(cfy+rgyn*LP) +(1 - AA) * accMAy
    cfrx = (cfx - cfxlast)/LP
    cfry = (cfy - cfylast)/LP#not used?
    cfxlast = cfx#not used
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
    Zx[1,0] =  rate_gyr_x
    
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
    Zy[1,0] =  rate_gyr_y
    
    Yy = Zy - H*Xy # find difference between sensors and model
    
    Sy = H*Py*H.T + R #get covariance 
    
    Ky = Py*H.T*Sx.I   #kalman gain
    
    Xy = Xy + Ky*Yy  #adjust guess based on kalman gain and sensor readings
    
    Py = (ID-Ky*H)*Py #fix covariance
    
    
    #####calibration2
    if wcal:  #calibration
        sensor[1] = Xy[0,0]
        sensor[0] = Xx[0,0]
        xoffset = offsetmult*xoffset+(1-offsetmult)*sensor[0]
        yoffset = offsetmult*yoffset+(1-offsetmult)*sensor[1]
        c1 = False
        
    else: #caliration complete
        sensor[1] = Xy[0,0]-yoffset
        sensor[0] = Xx[0,0]-xoffset
        
        cxavg = cxavg*clp + sensor[0]*(1-clp)#this is only used for led indicator light
        cyavg = cyavg*clp + sensor[1]*(1-clp)
        if abs(cxavg)<cth and abs(cyavg) < cth:
            c1 = True
        else:
            c1 = False
    
    GPIO.output(myled,c1)

##    LPEND = datetime.datetime.now()-start ###debug only or tie to LP
##    LPmic = LPEND.microseconds
##    print str(LPmic) +" is time after kalman  in miliSeconds" ###debug only 
    
    
##---error---------------------------------------------------------error

    error[0] = wpitch-sensor[0]
    error[1] = wroll -sensor[1]
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
    if active:
        base=dth
    else: 
        base = inactive
        
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
        
##        if nun:
##            throt[0] +=(wyaw*ky)#check if this behavior is correct
##            throt[1] -=(wyaw*ky)
##            throt[2] -=(wyaw*ky)
##            throt[3] +=(wyaw*ky)




####---pwm------------------------------------------------------------PWM
    #adafruit need "ADC" 
    throt[0] = int( throt[0] / Tonestep)
    throt[1] = int( throt[1] / Tonestep)
    throt[2] = int( throt[2] / Tonestep)
    throt[3] = int( throt[3] / Tonestep)
    
#adafruit
    if normal3:##esc calibraton mode off
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


    if loopcnt<loopmax:
        loopcnt= loopcnt+1
    else:
        loopcnt=0
        b = int(s1['battery']/10)
        if b>15:b=15
        elif b<1:b=1
        wii.led = b
        
##---wait---------------------------------------------------------wait
    LPcalc = time.clock()-start
    LPcalc2= time.time()-start2
    LPcalc3 = datetime.datetime.now()-start3
    LPcalc3 = LPcalc3.microseconds/1000000.0 
    print bool(EnableFixedLP and LPcalc<LP)
    if EnableFixedLP and LPcalc2<LP:
        time.sleep(LP-LPcalc2)
##---done---------------------------------------------------------done print stuff here to debug
    LPwhole = time.clock()-start
    LPwhole2 = time.time()-start2
    LPwhole3 = datetime.datetime.now()-start3
    LPwhole3 = LPwhole3.microseconds/1000000.0 
        
        
        
    print "lpcalc was %1.5f"%(LPcalc)
    print "lpcalc2 was %1.5f"%(LPcalc2)
    print "lpcalc3 was %1.5f"%(LPcalc3)
    print "lpwhole was %1.5f"%(LPwhole)
    print "lpwhole2 was %1.5f"%(LPwhole2)
    print "lpwhole3 was %1.5f"%(LPwhole3)
    print "throt0: %5i 1: %5i  2: %5i  3: %5i" %(int(throt[0]), int(throt[1]), int(throt[2]) , int(throt[3]))
    

    # #################printer debugg
    #print "FL: %3s  FR:  %3s   BL: %3s  BR: %3s" %(str(int(throt[0])), str(int(throt[1])), str(int(throt[2])), str(int(throt[3])))
##    if active ==1:
##        print "filterx: %5.3f   accxlp  %5.3f   gyrox %5.3f    " %(sensor[0],AccXLP,deltaXgyro)
    print "sensor x %3.5f      sensory %3.5f      "%(sensor[0], sensor[1] )
##    print "cfxr %3.5f   rgyrx %3.5f  Xx1 %-3.5f  Xx2 %-3.5f  y1 %3.5f      y2 %3.5f       "%(cfrx, rate_gyr_x ,Xx[1,0]/LP ,Xx[2,0]/LP,cfry , rate_gyr_y )
    print "accmax %3.5f      y %3.5f      "%(accMAx, accMAy )
    #print "kp %3.8f      ki %3.8f     kd %3.8f       "%(kp,ki,kd)
    print "kp: %3.5f  ki:  %3.5f   kd: %3.5f" %(kp, ki, kd )  
##        print "filterx: %5.3f   accxlp  %5.3f   gyrox %5.3f    " %(sensor[0],AccXLP,deltaXgyro)
    

    


##    print "e: %3.5f  de:  %3.5f   ie: %3.5f" %(error[0], derror[0], ierror[0] )
##    print "e: %3.5f  de:  %3.5f   ie: %3.5f" %(error[1], derror[1], ierror[1] )


    
    
