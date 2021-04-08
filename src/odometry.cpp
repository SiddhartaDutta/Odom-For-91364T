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
