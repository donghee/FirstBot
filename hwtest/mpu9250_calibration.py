import os
import sys
import time
import smbus
import time

from imusensor.MPU9250 import MPU9250

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()
#imu.setAccelRange("AccelRangeSelect2G")
#imu.setGyroRange("GyroRangeSelect250DPS")

#print("Gyro calibration starting")
#imu.caliberateGyro()
#print("Gyro calibration finished")
#print('gyro bias')
#print(imu.GyroBias)

#print("Accel calibration starting")
#imu.caliberateAccelerometer()
#print("Accel calibration finished")
#print('accel scale')
#print(imu.Accels)
#print('accel bias')
#print(imu.AccelBias)

#print("Mag calibration starting")
#time.sleep(2)
#imu.caliberateMagPrecise()
#print("Mag calibration finished")
#print('mag transformation matrix')
#print(imu.Magtransform)
#print('mag bias')
#print(imu.MagBias)
#print('mags')
#print (imu.Mags)

#imu.saveCalibDataToFile("/home/ubuntu/calib.json")
# or load your own caliberation file
#imu.loadCalibDataFromFile("/home/ubuntu/calib.json")

while True:
	imu.readSensor()
	imu.computeOrientation()

	print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
	time.sleep(0.1)
