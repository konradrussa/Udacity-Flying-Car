import numpy as np

data_gps = np.loadtxt('Graph1.txt', delimiter=',', dtype='Float64', skiprows=1)
data_gps = data_gps[:,1]

gps_mean = np.mean(data_gps,0)
print("GPS_MEAN: ", gps_mean)
print("GPS_MEAN2:", sum(data_gps)/len(data_gps))
print("GPS_STD:  ",np.std(data_gps))
gps_std = 0
for i in data_gps:
	gps_std += (i - gps_mean)**2
print("GPS_STD2: ",np.sqrt(gps_std/len(data_gps)))

data_imu = np.loadtxt('Graph2.txt', delimiter=',', dtype='Float64', skiprows=1)
data_imu = data_imu[:,1]

imu_mean = np.mean(data_imu,0)
print("IMU_MEAN: ", imu_mean)
print("IMU_MEAN2:", sum(data_imu)/len(data_imu))
print("IMU_STD:  ",np.std(data_imu))
imu_std = 0
for i in data_imu:
	imu_std += (i - imu_mean)**2
print("IMU_STD2: ",np.sqrt(imu_std/len(data_imu)))

#GPS_MEAN:  0.000483343434343
#GPS_MEAN2: 0.000483343434343
#GPS_STD:   0.7091878283
#GPS_STD2:  0.7091878283
#IMU_MEAN:  -0.0155729778114
#IMU_MEAN2: -0.0155729778114
#IMU_STD:   0.488252565593
#IMU_STD2:  0.488252565593