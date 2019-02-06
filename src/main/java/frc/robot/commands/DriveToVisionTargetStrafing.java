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

  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0};
  double startAngle;
  double[] visioninfo;
  double targetPos;
  double angleOffset;
  double targetAngle;
  double currentOffset;
  double currentDist;
  Phase currentPhase;
  boolean finished;

  public DriveToVisionTargetStrafing() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    visioninfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    targetPos = visioninfo[3];
    angleOffset = visioninfo[4] * (180/Math.PI);
    finished = false;
    currentPhase = Phase.DRIVE;
    SmartDashboard.putString("Thing is going", "yes");
    targetAngle = Robot.Drivetrain.getyaw() + angleOffset;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    visioninfo = SmartDashboard.getNumberArray("vision/target_info", empty);
    angleOffset = visioninfo[4] * (180/Math.PI);
    //System.out.println("Angle: " + angleOffset);
    
    switch (currentPhase){
      case DRIVE:
        //System.out.println(visioninfo[3]);
        if ((visioninfo[3] > RobotMap.AUTO_DRIVE_DISTANCE_THRESHOLD || (Math.abs(visioninfo[3]) < 0.1))){
          Robot.Drivetrain.alldrive(Robot.Drivetrain.driveSpeedCalc(visioninfo[3]), Math.signum(angleOffset) * Robot.Drivetrain.turnSpeedCalc(angleOffset), 0);
          System.out.println("DRIVE (" + visioninfo[3] + " in to go)");
          break;
        }
        currentPhase = Phase.FINAL_FACE_TARGET;
        break;

      case FINAL_FACE_TARGET:

        double deltaAngle = angleOffset + (visioninfo[5] * (180/Math.PI));
        System.out.println("deltaAngle: " + deltaAngle);
        if (Math.abs(deltaAngle) > RobotMap.AUTO_ANGLE_DIFFERENTIAL_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, Math.signum(deltaAngle) /* Robot.Drivetrain.turnSpeedCalc(angleOffset)*/ * 0.5, 0);
          break;
        }
        currentPhase = Phase.STRAFE_TO_ANGLE;
        break;
          

      case STRAFE_TO_ANGLE:
        System.out.println("STRAFE_TO_ANGLE");
        if (angleOffset > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD && visioninfo[1] > 0.5) {
          Robot.Drivetrain.alldrive(0, 0, 0.5);
          System.out.println("offset below threshold");
          break;
        } else if (angleOffset < -RobotMap.AUTO_TURN_ACCURACY_THRESHOLD &&  visioninfo[1] > 0.5) {
          Robot.Drivetrain.alldrive(0, 0, -0.5);
          System.out.println("offset above threshold");
          break;
        }
        else {
          finished = true;
          break;
        } 

      case TURN_TO_ANGLE:
        System.out.println("TURN_TO_ANGLE");
        if (Math.abs(angleOffset) > RobotMap.AUTO_TURN_ACCURACY_THRESHOLD) {
          Robot.Drivetrain.alldrive(0, Math.signum(angleOffset) * Robot.Drivetrain.turnSpeedCalc(angleOffset), 0);
          break;
        }
        currentPhase = Phase.DRIVE;
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
