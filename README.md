# QuadPi
this is a quadcopter that I am building with a raspi
it is still in development




#kalman filter constants:
r2 should be larger than r 1 since the gyro readings will not be affected by the quadrocopters acceleration. This will lead to the gyro responding quicker than the accelerometer.
q3 should be low since the gyro bias change extremely slow.
r1 should be larger than q1 to prevent acceleration from affecting the angle.
The relation between q2 and r2 will control how sensitive the angle output is to noise and how quick it will respond to changes.
