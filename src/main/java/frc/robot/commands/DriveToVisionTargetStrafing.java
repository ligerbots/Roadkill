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
    TURN_TO_ANGLE, STRAFE_TO_ANGLE, DRIVE
  }
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};

  double startAngle;
  double[] visioninfo;
  double targetPos;
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double currentDist;
  Phase currentPhase = Phase.TURN_TO_ANGLE;
  boolean finished;
  public DriveToVisionTargetStrafing() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    visioninfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    targetPos = visioninfo[3];
    angleOffset = visioninfo[4] * (180/Math.PI);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetAngle = Robot.Drivetrain.getyaw() + angleOffset;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visioninfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    angleOffset = visioninfo[4] * (180/Math.PI);
    System.out.println("Angle: " + angleOffset);
    
    switch (currentPhase){
      case TURN_TO_ANGLE:
        if (Math.abs(angleOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, -Math.signum(angleOffset) * Robot.Drivetrain.turnSpeedCalc(angleOffset), 0);
          break;
        }
        currentPhase = Phase.DRIVE;
        break;
      case STRAFE_TO_ANGLE:
        if (angleOffset > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, 0, 0.5);
          System.out.println("offset below threshold");
          break;
        } else if (angleOffset < -RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, 0, -0.5);
          System.out.println("offset above threshold");
          break;
        }
        finished = true;
        break;
      case DRIVE:
        if (visioninfo[3] < RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD){
          Robot.Drivetrain.alldrive(Robot.Drivetrain.driveSpeedCalc(visioninfo[3]), 0, 0);
          break;
        }
        currentPhase = Phase.STRAFE_TO_ANGLE;
        break;
      default:
        break;
    }

    /*visioninfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    targetPos = visioninfo[3];
    angleOffset = visioninfo[4] * (180/Math.PI);
    //targetAngle = Robot.Drivetrain.getyaw() + angleOffset;

    //currentOffset = targetAngle - Robot.Drivetrain.getyaw();
    
    System.out.println(currentPhase);
    System.out.println(currentOffset);
    
    switch (currentPhase) {
    case STRAFE_TO_ANGLE:
      // if (currentOffset > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD || currentDist <
      // RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD){
      if (angleOffset > -RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
        Robot.Drivetrain.alldrive(0, 0, 0.8);
        System.out.println("offset below threshold");
        break;
      } else if (angleOffset < RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
        Robot.Drivetrain.alldrive(0, 0, -0.8);
        System.out.println("offset above threshold");
        break;
      }
      currentPhase = Phase.DRIVE;
      break;
    case DRIVE:
      /*if (Robot.Drivetrain.getRobotPosition().distanceTo(targetPos) > RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD) {
        Robot.Drivetrain
            .alldrive(Robot.Drivetrain.driveSpeedCalc(Robot.Drivetrain.getRobotPosition().distanceTo(targetPos)), 0, 0);
        break;
      }
      finished = true;
      break;
      default:
      break;
    }
    System.out.println();*/
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
