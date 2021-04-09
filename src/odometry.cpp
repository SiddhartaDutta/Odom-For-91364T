/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odometry.cpp                                              */
/*    Author:       Siddharta Dutta                                           */
/*    Created:      Thu Apr 08 2021                                           */
/*    Description:  Contains all odometry algorithm definitions               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "chassis.h"
#include "odometry.h"

using namespace OdomLib;

/*----------------------------------------------------------------------------*/

	// Universal Chassis Object
Chassis chassis = compBot;

  // Angle and Point Definition
float x = 0, y = 0, heading = 0, relativeHeading = 0;

  // Stored Value Defintion
float prevLeft = 0.0, prevRight = 0.0, prevBack = 0.0, currentLeft = 0.0, currentRight = 0.0, currentBack = 0.0;

  // Enables/disables odometry thread
bool OdomLib::enable_odom = false;

/*----------------------------------------------------------------------------*/

	// Minimum turn velocity
float minTurnVel = 1;

	// P value for turn PD system
float kP_turn = 0.75;

	// D value for turn PD system
float kD_turn = 3;

	// Acceptable margin of error for turn PD system
float MoE_turn = 0.5;

	// Minimum turn velocity
float minDriveVel = 5;

	// P value for drive PD system
float kP_drive = 95;

	// D value for drive PD system
float kD_drive = 0;

	// Acceptable margin of error for drive PD system
float MoE_drive = 0.1;

/*----------------------------------------------------------------------------*/

/**
* @brief Clears all odometry relavant values to default (0) and sets inputted values (if provided) to the inputted parameter.
* @param x Defaulted to 0; sets x to input value or to 0.
* @param y Defaulted to 0; sets y to input value or to 0.
* @param heading Defaulted to 0; sets heading to input value or to 0.
*/
void OdomLib::setOdometry(float set_x, float set_y, float set_heading){
	
		// Sets key odom values to inputs or default (0)
  x = set_x;
  y = set_y;
  heading = set_heading;

		// Sets all values to default (0)
  relativeHeading = 0;
  prev_Back = 0;
  prev_Left = 0;
  prev_Right = 0;
  current_Back = 0;
  current_Left = 0;
  current_Right = 0;
  
		// Resets all encoder values
  LEncoder.resetRotation();
  REncoder.resetRotation();
  BEncoder.resetRotation();
}

void OdomLib::clearEncoders(){
  LEncoder.resetRotation();
  REncoder.resetRotation();
  BEncoder.resetRotation();
}

/**
* @brief Reduces input angle to max range of [-(PI), (PI)] radians or [-180, 180] degrees.
* @return Returns a float that is the angle converted to the reduced angle range (in degrees).
* @param angle_deg The angle measure (in degrees) to be converted.
*/
float reduceAngle(float angle_deg){

  while(!(angle_deg >= -180 && angle_deg < 180)){

    if(angle_deg < -180){ angle_deg += 360; }
    if(angle_deg >= 180){ angle_deg -= 360; }
    
  }

  return angle_deg;
}

/**
* @brief Reduces input angle to max range of [0, 2(PI)] radians or [0, 360] degrees.
* @return Returns a float that is the angle converted to the reduced angle range (in radians).
* @param angle_rad The angle measure (in radians) to be converted.
*/
float normalizeAngle(float angle_rad){

  while( !(angle_rad >= 0 && angle_rad < (2 * M_PI)) ){

    if(angle_rad < 0){ angle_rad += (2 * M_PI); }
    if(angle_rad >= (2 * M_PI)){ angle_rad -= (2 * M_PI); }

  }

  return angle_rad;
}

/**
* @brief Sets motor velocities to the identified params and spins the motors at their respective velocities.
* @param LFSpeed Velocity input for the left front motor.
* @param LBSpeed Velocity input for the left back motor.
* @param RFSpeed Velocity input for the right front motor.
* @param RBSpeed Velocity input for the right back motor.
*/
void setDriveVelocity(float LFSpeed, float LBSpeed, float RFSpeed, float RBSpeed){
  LDriveF.spin(directionType::fwd, LFSpeed, velocityUnits::pct);
  LDriveB.spin(directionType::fwd, LBSpeed, velocityUnits::pct);
  RDriveF.spin(directionType::fwd, RFSpeed, velocityUnits::pct);
  RDriveB.spin(directionType::fwd, RBSpeed, velocityUnits::pct);
}

/**
* @brief Sets all motor velocities to 0, stopping the motors.
*/
void stopDrive(){
  LDriveF.stop(brakeType::hold);
  LDriveB.stop(brakeType::hold);
  RDriveF.stop(brakeType::hold);
  RDriveB.stop(brakeType::hold);
  wait(50, msec);
  LDriveF.stop(brakeType::coast);
  LDriveB.stop(brakeType::coast);
  RDriveF.stop(brakeType::coast);
  RDriveB.stop(brakeType::coast);
}

/*----------------------------------------------------------------------------*/

/**
* @brief A task that updates attitude when run.
* @return Returns a float that is the angle converted to the reduced angle range (in radians).
* @param angle_rad The angle measure (in radians) to be converted.
*/
int OdomLib::updatePosition(){

  clearEncoders();
  setOdometry();

  while(enable_odom){  

    current_Left = -LEncoder.position(rotationUnits::rev);
    current_Right = REncoder.position(rotationUnits::rev);
    current_Back = -BEncoder.position(rotationUnits::rev);

    float dL = current_Left * M_PI * chassis.getTWDia();
    float dR = current_Right * M_PI * chassis.getTWDia();
    float dB = current_Back * M_PI * chassis.getTWDia();

    float dHeading = (dL - dR) / (chassis.getLDist() + chassis.getRDist());

    float h;
    float halfAngle;
    float h2;

    if(dHeading){

      float radius = dR / dHeading;
      halfAngle = dHeading / 2;

      float halfSin = sin(halfAngle);
      h = ((radius + chassis.getRDist()) * halfSin) * 2.0;

      float radius2 = dB / dHeading;
      h2 = ((radius2 + chassis.getBDist()) * halfSin) * 2.0;

    } else {

      h = dR;
      halfAngle = 0;
      h2 = dB;

    }

    float endAngle = halfAngle + heading;
    float cosA = cos(endAngle);
    float sinA = sin(endAngle);

    y += h * cosA;
    x += h * sinA;

    y += h2 * -sinA;
    x += h2 * cosA;
  
    heading += dHeading;

    heading = normalizeAngle(heading);

    clearEncoders();

    #pragma region oldOdom
  /*
    //float temp_heading = ()
    
    float dL = (current_Left - prev_Left) * (M_PI / 180) * (chassis.getTWDia() / 2);
    float dR = (current_Right - prev_Right) * (M_PI / 180) * (chassis.getTWDia() / 2);
    float dB = (current_Back - prev_Back) * (M_PI / 180) * (chassis.getTWDia() / 2);

    prev_Left = current_Left;
    prev_Right = current_Right;
    prev_Back = current_Back;

    float h;
    float halfAngle;
    float h2;

    relativeHeading = (dL - dR) / (chassis.getLDist() + chassis.getRDist());

    if(relativeHeading){

      float radius = dR / relativeHeading;
      halfAngle = relativeHeading / 2;

      float halfSin = sin(halfAngle);
      h = ((radius + chassis.getRDist()) * halfSin) * 2.0;

      float radius2 = dB / relativeHeading;
      h2 = ((radius2 + chassis.getBDist()) * halfSin) * 2.0;

    } else {

      h = dR;
      halfAngle = 0;
      h2 = dB;

    }

    float endAngle = halfAngle + heading;
    float cosA = cos(endAngle);
    float sinA = sin(endAngle);

    y += h * cosA;
    x += h * sinA;

    y += h2 * -sinA;
    x += h2 * cosA;

    heading += relativeHeading;

    heading = normalizeAngle(heading);
    */
    #pragma endregion

    #pragma region Brain_Update
      Brain.Screen.clearScreen();
      Brain.Screen.printAt(10, 20, "X: %f", getXPos());
      Brain.Screen.printAt(10, 70, "Y: %f", getYPos());
      Brain.Screen.printAt(10, 120, "Heading: %f", getHeadingDeg());
      Brain.Screen.printAt(10, 170, "Controller Ax3 %d", Controller.Axis3.value());
      Brain.Screen.printAt(10, 230, "Controller Ax2 %d", Controller.Axis2.value());
    #pragma endregion

    task::sleep(5);
  }

  return 1;
}

/**
* @brief Accessor-type function to return calculated x position.
* @return Returns a float that is the calculated x position of the robot.
*/
float OdomLib::getXPos(){ return x; }

/**
* @brief Accessor-type function to return calculated y position.
* @return Returns a float that is the calculated y position of the robot.
*/
float OdomLib::getYPos(){ return y; }

/**
* @brief Accessor-type function to return calculated heading (in radians).
* @return Returns a float that is the calculated heading of the robot (in radians).
*/
float OdomLib::getHeading(){ return heading; }

/**
* @brief Accessor-type function to return calculated heading (in degrees).
* @return Returns a float that is the calculated heading of the robot (in degrees).
*/
float OdomLib::getHeadingDeg(){ return heading * 180 / M_PI; }

#pragma endregion



#pragma region Motion_Control

/**
* @brief Turns robot to orient towards target heading.
* @param target_angle_deg The angle to be oriented towards.
*/
void pointTurn(float target_angle_deg){

  setOdometry(0, 0, 0);

  float tempDeg = getHeadingDeg();

  float turnError = reduceAngle(fabs(target_angle_deg) - tempDeg);

  int turnDirec = fabs(target_angle_deg)/target_angle_deg;

  std::cout << "TurnDirec: " << turnDirec << std::endl;

  float deltaTurnError = 0;

  float prevTurnError = turnError;

    // Keeps turning the robot until the error is within the defined range of accepted error.
  while( fabs( turnError ) > turn_MoE){

    tempDeg = getHeadingDeg();

    if(getHeadingDeg() > target_angle_deg && turnDirec < 0){
      tempDeg = 360 - getHeadingDeg();
    }

    turnError = reduceAngle(fabs(target_angle_deg) - tempDeg);

    deltaTurnError = turnError - prevTurnError;

    prevTurnError = turnError;

    float turnSpeed = (turnError * turnP) + (deltaTurnError * turnD);

    std::cout << "Speed: " << turnSpeed << std::endl;

    if(turnSpeed < minTurnVel){
      std::cout << "break | " << turnError << std::endl;
      break;
    }

    setDriveVelocity(turnDirec * turnSpeed, turnDirec * turnSpeed, turnDirec * -turnSpeed, turnDirec * -turnSpeed);

    wait(5, msec);
  }

  std::cout << "done | " << turnError << std::endl;
  stopDrive();

}

/**
* @brief Moves robot to a specified point on the field without changing heading.
* @param target_x The target x-coordinate to be driven to.
* @param target_y The target y-coordinate to be driven to.
*/
void pointDrive(float target_x, float target_y){

}

/**
* @brief Moves robot to a specified point on the field and simultaneously changes heading.
* @param target_x The target x-coordinate to be driven to.
* @param target_y The target y-coordinate to be driven to.
* @param target_angle_deg The angle to be oriented towards.
*/
void moveTo(float target_x, float target_y, float target_angle_deg){

}


void driveTo(float targetIn){

  setOdometry(0, 0, 0);

  float driveError = 1 - (getYPos() / targetIn);

  float driveDirec = fabs(targetIn) / targetIn;

  float deltaDriveError = 0;
  float prevDriveError = driveError;

  while(driveError > drive_MoE){
    driveError = 1 - (getYPos() / targetIn);

    deltaDriveError = driveError - prevDriveError;

    prevDriveError = driveError;

    float driveSpeed = (driveError * driveP) + (driveError * driveD);

    std::cout << "Speed: " << driveSpeed << std::endl;

    if(driveSpeed < minDriveVel){
      std::cout << "break | " << driveError << std::endl;
      break;
    }

    setDriveVelocity(driveDirec * driveSpeed, driveDirec * driveSpeed, driveDirec * driveSpeed, driveDirec * driveSpeed);
    
    wait(5, msec);
  }

  std::cout << "done | " << driveError << std::endl;
  stopDrive();
}

void encoderTest(){
  // 12
  enable_odom = false;
  LEncoder.resetRotation();
  REncoder.resetRotation();
  float target = 5;
  float driveErrorL = target - (-LEncoder.rotation(rotationUnits::rev) * chassis.getTWDia() * M_PI);
  float driveErrorR = target - (REncoder.rotation(rotationUnits::rev) * chassis.getTWDia() * M_PI);

  while(driveErrorL > 1){
    driveErrorL = target - (-LEncoder.rotation(rotationUnits::rev) * chassis.getTWDia() * M_PI);
    std::cout << driveErrorL << std::endl;
    setDriveVelocity(driveErrorL * 10, driveErrorL * 10, 0, 0);
    wait(5, msec);
  }

  stopDrive();
  std::cout << "otra wei\n";

  while(driveErrorR > 1){
    driveErrorR = target - (REncoder.rotation(rotationUnits::rev) * chassis.getTWDia() * M_PI);
    std::cout << driveErrorR << std::endl;
    setDriveVelocity(0, 0, driveErrorR * 10, driveErrorR * 10);
    wait(5, msec);
  }

  stopDrive();
  enable_odom = true;
}

void funcTest(){
  //setOdometry(0, 0, 0);
  //pointTurn(45);
  //driveTo(-12);
  encoderTest();
  std::cout << "switch\n";
  //driveTo(-10);
  
}

#pragma endregion
