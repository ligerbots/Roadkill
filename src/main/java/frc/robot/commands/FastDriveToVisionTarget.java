/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FastDriveToVisionTarget extends Command {

  double[] visionInfo;                                          //holds info recieved from the Odroid through NT
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};      //empty array of values to be used for default value when fetching
  double[] previousInfo;

  double distance;                           //total distance (raw from NT) from robot to target
  double angle;                              //angle from robot to target in degrees (NT is initially in radians)
  double deltaAngle;

  public FastDriveToVisionTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3];                                                 //reset distance and angle
    angle = visionInfo[4] * (180/Math.PI);
    deltaAngle = angle + (visionInfo[5] * (180/Math.PI));


    Robot.Drivetrain.alldrive(Robot.Drivetrain.driveSpeedCalc(distance), Robot.Drivetrain.turnSpeedCalc(deltaAngle) * Math.signum(deltaAngle), Math.signum(angle) * Robot.Drivetrain.strafeSpeedCalc(angle));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
   // return distance <= RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD && Math.abs(angle) <= RobotMap.AUTO_TURN_ACCURACY_THRESHOLD && Math.abs(deltaAngle) < RobotMap.AUTO_ANGLE_DIFFERENTIAL_THRESHOLD;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("FINISHED");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
