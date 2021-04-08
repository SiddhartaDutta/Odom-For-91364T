/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odometry.h                                                */
/*    Author:       Siddharta Dutta                                           */
/*    Created:      Thu Apr 08 2021                                           */
/*    Description:  Contains all odometry algorithm declarations              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef ODOMETRY_H
#define ODOMETRY_H

namespace OdomLib {
  
    // Enables/Disables odometry task
  extern bool enable_odom;
  
    // Function for odometry task
  int updatePosition();
  
    // Functions for accessing coordinates and heading
  float getXPos();
  float getYPos();
  float getHeading();
  float getHeadingDeg();
  
    // Clears all odometry relavant values to default (0) and sets inputted values (if provided)
  void setOdometry(float setX = 0, float setY = 0, float setHeading = 0);
  
    // Clears all odometry encoders
  void clearEncoders();
  
    // Turns to given heading
  void pointTurn(float deg);
  
    // MAJOR ISSUE: WORKS EXCLUSIVELY IN Y-DIREC 
  void driveTo(float target);
  
}

#endif
