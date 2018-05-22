#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

float alt_previous_error = 0; // new variable for D in altitude

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  float l = L / sqrt(2.0f); // perpendicular distance to axes

  //collThrustCmd = f1 + f2 + f3 + f4;
  //momentCmd.y = l * (f1 + f2 - f3 - f4);
  //momentCmd.z = kappa * f1 - kappa * f2 + kappa * f3 - kappa * f4;

  float f1 = (kappa * (collThrustCmd * l + momentCmd.x + momentCmd.y) + l * momentCmd.z) / (4.0f * l * kappa);
  float f2 = ((l * collThrustCmd - momentCmd.x + momentCmd.y) / l - momentCmd.z / kappa) / 4.0f;
  float f3 = (l * kappa * collThrustCmd + l * momentCmd.z - kappa * momentCmd.x - kappa * momentCmd.y) / (4.0f * l * kappa);
  float f4 = ((l * collThrustCmd + momentCmd.x - momentCmd.y) / l - momentCmd.z / kappa) / 4.0f;

  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
  //collThrustCmd / 4 for altitude

  // f1 CW , f2CCW
  // f4CCW, f3 CW

  cmd.desiredThrustsN[0] = CONSTRAIN(f1, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(f2, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(f4, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(f3, minMotorThrust, maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  // Convert desired moment into differential thrusts
  V3F diffThrust;

  // for X shaped quad
  diffThrust.x = momentCmd.x / L / 2.f / sqrtf(2);
  diffThrust.y = momentCmd.y / L / 2.f / sqrtf(2);
  diffThrust.z = momentCmd.z / 4.f / kappa;

  // MIXING
  // combine the collective thrust with the differential thrust commands to find desired motor thrusts
  // X Shaped Quad (NED Frame)
  cmd.desiredThrustsN[0] = collThrustCmd / 4.f - diffThrust.z + diffThrust.y + diffThrust.x; // front left
  cmd.desiredThrustsN[1] = collThrustCmd / 4.f + diffThrust.z + diffThrust.y - diffThrust.x; // front right
  cmd.desiredThrustsN[2] = collThrustCmd / 4.f + diffThrust.z - diffThrust.y + diffThrust.x; // rear left
  cmd.desiredThrustsN[3] = collThrustCmd / 4.f - diffThrust.z - diffThrust.y - diffThrust.x; // rear right
  
  //////////////////////////////// END SOLUTION ///////////////////////////////
  */
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F pqr_error = pqrCmd - pqr;

  float u_bar_p = kpPQR.x * pqr_error.x * Ixx;
  float u_bar_q = kpPQR.y * pqr_error.y * Iyy;
  float u_bar_r = kpPQR.z * pqr_error.z * Izz;

  momentCmd = V3F(u_bar_p, u_bar_q, u_bar_r);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////
  V3F rate_error = pqrCmd - pqr;
  V3F omega_dot_des = rate_error * kpPQR;
  momentCmd = omega_dot_des * V3F(Ixx, Iyy, Izz);
  //////////////////////////////// END SOLUTION ///////////////////////////////
  */

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float b_x = R(0, 2);
  float b_x_err = CONSTRAIN(accelCmd[0] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle) - b_x;
  float b_x_p_term = kpBank * b_x_err;

  float b_y = R(1, 2);
  float b_y_err = CONSTRAIN(accelCmd[1] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle) - b_y;
  float b_y_p_term = kpBank * b_y_err;

  float rot_formula[2][2] = { { R(1, 0) / R(2, 2), -R(0, 0) / R(2, 2) },{ R(1, 1) / R(2, 2), -R(0, 1) / R(2, 2) } };
  float p_c = rot_formula[0][0] * b_x_p_term + rot_formula[0][1] * b_y_p_term;
  float q_c = rot_formula[1][0] * b_x_p_term + rot_formula[1][1] * b_y_p_term;
  pqrCmd = V3F(p_c, q_c, 0.0f);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////

  
  float target_R13 = -CONSTRAIN(accelCmd[0] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
  float target_R23 = -CONSTRAIN(accelCmd[1] / (collThrustCmd / mass), -maxTiltAngle, maxTiltAngle);
    
  if (collThrustCmd < 0)
  {
    target_R13 = 0;
    target_R23 = 0;
  }
  pqrCmd.x = (1 / R(2, 2))*(-R(1, 0) * kpBank*(R(0, 2) - target_R13) + R(0, 0) * kpBank*(R(1, 2) - target_R23));
  pqrCmd.y = (1 / R(2, 2))*(-R(1, 1) * kpBank*(R(0, 2) - target_R13) + R(0, 1) * kpBank*(R(1, 2) - target_R23));

  //////////////////////////////// END SOLUTION ///////////////////////////////
  */

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float z_err = posZCmd - posZ;
  float z_err_dot_d = (z_err - alt_previous_error) / dt;
  float z_err_dot = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate) - velZ;
  integratedAltitudeError += z_err * dt;
  alt_previous_error = z_err;

  float p_term = kpPosZ * z_err;
  float d_term = kpVelZ * z_err_dot + 0.04f * z_err_dot_d; // added small adjustment using previous error rate
  float i_term = KiPosZ * integratedAltitudeError;

  float u_1_bar = p_term + d_term + i_term + accelZCmd;

  float b_z = R(2, 2);
  float u_1 = (u_1_bar - 9.81f) / b_z;
  thrust = -u_1 * mass;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////

  velZCmd += kpPosZ * (posZCmd - posZ);

  integratedAltitudeError += (posZCmd - posZ) * dt;

  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  float desAccel = kpVelZ * (velZCmd - velZ) + KiPosZ * integratedAltitudeError + accelZCmd - 9.81f;

  thrust = -(desAccel / R(2, 2) * mass);

  //////////////////////////////// END SOLUTION ///////////////////////////////
  */

  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float x_err = posCmd.x - pos.x;
  float x_err_dot = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY) - vel.x;        // fmodf(velCmd.x - vel.x, maxSpeedXY);
  float x_dot_dot_target = kpPosXY * x_err + kpVelXY * x_err_dot + accelCmd.x;

  float y_err = posCmd.y - pos.y;
  float y_err_dot = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY) - vel.y;        // fmodf(velCmd.y - vel.y, maxSpeedXY);
  float y_dot_dot_target = kpPosXY * y_err + kpVelXY * y_err_dot + accelCmd.y;

  accelCmd.x = -CONSTRAIN(x_dot_dot_target, -maxAccelXY, maxAccelXY);            // -fmodf(x_dot_dot_target, maxAccelXY);
  accelCmd.y = -CONSTRAIN(y_dot_dot_target, -maxAccelXY, maxAccelXY);            // -fmodf(y_dot_dot_target, maxAccelXY);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////

  velCmd += kpPosXY * (posCmd - pos);

  if (velCmd.mag() > maxSpeedXY)
  {
    velCmd = velCmd * maxSpeedXY / velCmd.mag();
  }

  accelCmd += kpVelXY * (velCmd - vel);
  if (accelCmd.mag() > maxAccelXY)
  {
    accelCmd = accelCmd * maxAccelXY / accelCmd.mag();
  }

  //////////////////////////////// END SOLUTION ///////////////////////////////
  */

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  yawCmd = fmod(yawCmd, 2.0f * M_PI);
  float yaw_err = yawCmd - yaw;
  if (yaw_err > M_PI) {
	  yaw_err = yaw_err - 2.0f * M_PI;
  }
  else if (yaw_err < -M_PI) {
	  yaw_err = yaw_err + 2.0f * M_PI;
  }
  yawRateCmd = kpYaw * yaw_err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  /*
  /////////////////////////////// BEGIN SOLUTION //////////////////////////////

  float yawError = yawCmd - yaw;
  yawError = fmodf(yawError, F_PI*2.f);
  if (yawError > F_PI)
  {
    yawError -= 2.f * F_PI;
  }
  else if (yawError < -F_PI)
  {
    yawError += 2.f * F_PI;
  }
  yawRateCmd = yawError * kpYaw;

  //////////////////////////////// END SOLUTION ///////////////////////////////
  */

  return yawRateCmd;
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
