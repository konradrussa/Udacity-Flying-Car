

I needed to fix lateral position and and re-tune PID controller to obtain +- velocities and accelerations using CONSTRAIN, 
previously using fmod and constant flow or not was not enough :), actually that was a bug, and not allow to successfully run simulator with estimation and stop drone on time.

Criteria:

----------------------------------------------------------------------------------------
Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data:

i have built this script:
```
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
```
where 
GPS_STD:   0.7091878283
IMU_STD:   0.488252565593

----------------------------------------------------------------------------------------

Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.

Implementation:
```
  /*
  // Attitude Update
  //USING Rotation Matrix and Euler Forward method to get Euler angles at time t
  //Rotation Matrix for gyro, used body frame
  //	  1, sin(phi)*tan(theta), cos(phi)*tan(theta)
  //	  0, cos(phi)           , -sin(phi)
  //	  0, sin(phi)/cos(theta), cos(phi)/cos(theta)

  const float v[9] = { 1, sin(rollEst)*tan(pitchEst)  , cos(rollEst)*tan(pitchEst),
					   0, cos(rollEst)             , -sin(rollEst),
					   0, sin(rollEst) / cos(pitchEst), cos(rollEst) / cos(pitchEst) };
  Mat3x3F rot_mat = Mat3x3F(v);
  V3F timeDerivativeOfEuler = rot_mat * gyro;

  float predictedRoll = rollEst + timeDerivativeOfEuler.x * dtIMU;
  float predictedPitch = pitchEst + timeDerivativeOfEuler.y * dtIMU;
  ekfState(6) += timeDerivativeOfEuler.z * dtIMU;
  */

  //USING Quaternion to get orientation
  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
  attitude.IntegrateBodyRate(gyro, dtIMU); //Gyro Integration - Dead Reckoning
  float predictedRoll = attitude.Roll();
  float predictedPitch = attitude.Pitch();
  ekfState(6) = attitude.Yaw();

  // normalize yaw to -pi .. pi
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
  ```
----------------------------------------------------------------------------------------

Implement all of the elements of the prediction step for the estimator.

PredictState:
```
  //valocity integration to update position
  predictedState(0) += ekfState(3) * dt;
  predictedState(1) += ekfState(4) * dt;
  predictedState(2) += ekfState(5) * dt;

  attitude.IntegrateBodyRate(gyro, dtIMU); //integrate body rates

  V3F global_acceleration = attitude.Rotate_BtoI(accel); // IMU gives body frame, required transformation to Inertial frame
  global_acceleration.z -= 9.81f;						 // IMU does not include gravity, including gravity component

  //acceleration integration to update velocity
  predictedState(3) += global_acceleration.x * dt;
  predictedState(4) += global_acceleration.y * dt;
  predictedState(5) += global_acceleration.z * dt;
 
GetRbgPrime:
  /*
  R'bg =
		  -cos(theta)*cos(psi), -sin(phi)*sin(theta)*sin(psi) - cos(theta)*cos(psi), -cos(phi)*sin(theta)*sin(psi) + sin(phi)*cos(psi)
		  cos(theta)*cos(psi) , sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)   , cos(phi)*sin(theta)*sin(psi) - sin(phi)*sin(psi)
		  0                   , 0                                                  , 0
  */

  RbgPrime(0, 0) = -cos(pitch)*cos(yaw);
  RbgPrime(0, 1) = -sin(roll)*sin(pitch)*sin(yaw) - cos(pitch)*cos(yaw);
  RbgPrime(0, 2) = -cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw);
  RbgPrime(1, 0) = cos(pitch)*cos(yaw);
  RbgPrime(1, 1) = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
  RbgPrime(1, 2) = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*sin(yaw);

Predict:
  //Jacobian of transition model
  //      1  0  0  dt 0  0  0 
  //	  0  1  0  0  dt 0  0
  //	  0  0  1  0  0  dt 0
  //	  0  0  0  1  0  0  RbgPrime[0:]ut[0:3]dt
  //	  0  0  0  0  1  0  RbgPrime[1:]ut[0:3]dt
  //	  0  0  0  0  0  1  RbgPrime[2:]ut[0:3]dt
  //	  0  0  0  0  0  0  1

  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;

  V3F j1 = RbgPrime(0) * accel * dt;
  V3F j2 = RbgPrime(1) * accel * dt;
  V3F j3 = RbgPrime(2) * accel * dt;

  gPrime(3, 6) = j1.mag();
  gPrime(4, 6) = j2.mag();
  gPrime(5, 6) = j3.mag();
  
  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;  
```
----------------------------------------------------------------------------------------

Implement the magnetometer update.
```
  float yaw_angle_diff = z(0) - ekfState(6);

  // normalize yaw to -pi .. pi
  if (yaw_angle_diff < -F_PI) z(0) += 2.f*F_PI;
  if (yaw_angle_diff > F_PI) z(0) -= 2.f*F_PI;
  
  zFromX(0) = ekfState(6); // takes yaw estimation
  
  // hPrime partial derivative, Jacobian for yaw:
  //		[0, 0, 0, 0, 0, 0, 1]

  hPrime(0, 6) = 1;
  ```
----------------------------------------------------------------------------------------

Implement the GPS update.
```
  // hPrime - Jacobian, partial derivative for position and velocity, yaw not accounted:
  //	   [[1, 0, 0, 0, 0, 0, 0]
  //		[0, 1, 0, 0, 0, 0, 0]
  //		[0, 0, 1, 0, 0, 0, 0]
  //		[0, 0, 0, 1, 0, 0, 0]
  //		[0, 0, 0, 0, 1, 0, 0]
  //		[0, 0, 0, 0, 0, 1, 0]]

  for (int i = 0; i < 6; i++) {
	  zFromX(i) = ekfState(i); // updates for positions and velocities estimations
	  hPrime(i, i) = 1;
  }
```
