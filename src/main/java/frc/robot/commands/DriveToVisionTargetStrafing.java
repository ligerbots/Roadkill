/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToVisionTargetStrafing extends Command {
  
  enum Phase {
    TURN_TO_ANGLE, STRAFE_TO_ANGLE, DRIVE, FINAL_FACE_TARGET
  }

  double[] visionInfo;                                          //holds info recieved from the Odroid through NT
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};      //empty array of values to be used for default value when fetching
  double[] previousInfo;

  double distance;                           //total distance (raw from NT) from robot to target
  double angle;                              //angle from robot to target in degrees (NT is initially in radians)
  double overallAngle;                       //angle to the target in terms of the robot's yaw
  Phase currentPhase = Phase.DRIVE;          //keeps track of the current phase the command is in -- START IN DRIVE PHASE
  boolean finished = false;                  //when finished = true, the command will end

  public DriveToVisionTargetStrafing() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Switching to DRIVING phase");
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    previousInfo = visionInfo;
    distance = visionInfo[3];
    angle = visionInfo[4] * (180/Math.PI);
    overallAngle = Robot.Drivetrain.getyaw() + angle;
  }

  protected boolean isVisionInfoAccurate() {     //returns true for true failure of data and false for true success
    // if either the current info failed, or the change in angle and distance from the previous info is too great 
    // (making sure the previous info was also marked by the Odroid as a success), return false
    if (visionInfo[1] < 0.5 || visionInfo[2] >= 2.5 || (previousInfo[1] > 0.5 && visionInfo[3] - previousInfo[3] < 20.0 && visionInfo[4] - previousInfo[4] < 20.0)) {
      return false;            //TODO when the finder_id is changed on the odroid to be in order, must update this statement
    }
    return true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty);  //refetch value
    distance = visionInfo[3];                                                 //reset distance and angle
    angle = visionInfo[4] * (180/Math.PI);
    
    switch (currentPhase) {
      case DRIVE:

        System.out.println("DRIVE -- distance: " + distance + " in , angle: " + angle + " deg , drive speed: " + Robot.Drivetrain.driveSpeedCalc(distance) + " , turn speed: " + Robot.Drivetrain.turnSpeedCalc(angle));

        if (!isVisionInfoAccurate()) break;

        if (distance > RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD || distance < 0.1 ) { 
          Robot.Drivetrain.alldrive(Robot.Drivetrain.driveSpeedCalc(distance), Robot.Drivetrain.turnSpeedCalc(angle), 0);
          break;
        }
        currentPhase = Phase.FINAL_FACE_TARGET;
        System.out.println("Switching to FINAL_FACE_TARGET phase");
        break;

      case FINAL_FACE_TARGET:

        if (!isVisionInfoAccurate()) break;

        double deltaAngle = angle + (visionInfo[5] * (180/Math.PI));
        //System.out.println("deltaAngle: " + deltaAngle);
        if (Math.abs(deltaAngle) > RobotMap.AUTO_ANGLE_DIFFERENTIAL_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, Math.signum(deltaAngle) /* Robot.Drivetrain.turnSpeedCalc(angleOffset)*/ * 0.5, 0);
          break;
        }
        currentPhase = Phase.STRAFE_TO_ANGLE;
        System.out.println("Switching to STRAFE_TO_ANGLE phase");
        break;
          

      case STRAFE_TO_ANGLE:

        if (!isVisionInfoAccurate()) break;

        if (angle > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD && visionInfo[1] > 0.5) {
          Robot.Drivetrain.alldrive(0, 0, 0.5);
          break;
        } else if (angle < -RobotMap.AUTO_TURN_ACCURACY_THRESHOLD &&  visionInfo[1] > 0.5) {
          Robot.Drivetrain.alldrive(0, 0, -0.5);
          break;
        }
        else {
          finished = true;
          System.out.println("Switching to FINISH phase");
          break;
        } 

      case TURN_TO_ANGLE:
      
        if (!isVisionInfoAccurate()) break;

        if (Math.abs(angle) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, Math.signum(angle) * Robot.Drivetrain.turnSpeedCalc(angle), 0);
          break;
        }
        currentPhase = Phase.DRIVE;
        System.out.println("Switching to DRIVE phase");
        break;
      
      
      default:
        break;
      
    }

    previousInfo = visionInfo;    //reset previousInfo
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
