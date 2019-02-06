/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldPosition;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  enum DriveSide {
    LEFT, RIGHT, CENTER
  }

  boolean isfieldcentric = false;
  WPI_TalonSRX leftmaster;
  WPI_TalonSRX leftslave;
  WPI_TalonSRX rightmaster;
  WPI_TalonSRX rightslave;
  WPI_TalonSRX centermaster;
  WPI_TalonSRX centerslave;
  PIDController turningController;
  AHRS navx;
  DifferentialDrive RobotDrive;
  StickyFaults centermasterfaults;
  //boolean centerpresent = (centermaster.getStickyFaults(centermasterfaults) == ErrorCode.OK);
  private double turnOutput;

  public drivetrain() {
    navx = new AHRS(Port.kMXP, (byte) 200);
    leftmaster = new WPI_TalonSRX(RobotMap.ct_left_1);
    leftslave = new WPI_TalonSRX(RobotMap.ct_left_2);
    rightmaster = new WPI_TalonSRX(RobotMap.ct_right_1);
    rightslave = new WPI_TalonSRX(RobotMap.ct_right_2);
    centermaster = new WPI_TalonSRX(RobotMap.ct_center_1);
    centerslave = new WPI_TalonSRX(RobotMap.ct_center_2);
    centerslave.set(ControlMode.Follower, centermaster.getDeviceID());
    leftslave.set(ControlMode.Follower, leftmaster.getDeviceID());
    rightslave.set(ControlMode.Follower, rightmaster.getDeviceID());

    leftmaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    rightmaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    RobotDrive = new DifferentialDrive(leftmaster, rightmaster);

    turningController = new PIDController(0.045, 0.004, 0.06, navx, output -> this.turnOutput = output);


  }
  
  public void alldrive(double throttle, double rotation, double strafe){
    if (isfieldcentric){
      RobotDrive.arcadeDrive(throttle * Math.cos(getyaw()) + strafe * Math.sin(getyaw()), -rotation);
      centermaster.set(ControlMode.PercentOutput, (-throttle * Math.sin(getyaw()) + strafe * Math.cos(getyaw())));
    }
    else{
      RobotDrive.arcadeDrive(throttle, -rotation);
      centermaster.set(ControlMode.PercentOutput, strafe);
    }
    SmartDashboard.putNumber("yaw", getyaw());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    navx.zeroYaw();
   }

  public boolean isfieldcentric(){
    return isfieldcentric;
  }

  public void setfieldcentric(Boolean set){
    isfieldcentric = set;
  }

  public double getyaw(){
    return navx.getYaw() / 180 * Math.PI;
  }

 /* public double getEncoderDistance (DriveSide driveSide) {
    switch (driveSide) {
      case LEFT:
        return leftmaster.getEncoder().getPosition() / 42 /*I think?*/ /**
          RobotMap.SIDE_GEAR_RATIO * RobotMap.SIDE_WHEEL_DIAMETER;
      case RIGHT:
        return leftmaster.getEncoder().getPosition() / 42 /*I think?*/ //*
          /*RobotMap.SIDE_GEAR_RATIO * RobotMap.SIDE_WHEEL_DIAMETER;
      case CENTER:
        return leftmaster.getEncoder().getPosition() / 42 /*I think?*/ //*
         /* RobotMap.CENTER_GEAR_RATIO * RobotMap.CENTER_WHEEL_DIAMETER;
      default:
        return 0.0;
    }
  }*/

  public double turnSpeedCalc (double error) {
    if (error > 30) {return 0.8;}
    else if (error > 10) {return 0.8;}
    return 0.8;
  }

  public double driveSpeedCalc (double error) {
    if (error > 24) {return 0.8;}
    else if (error > 12) {return 0.6;}
    return 0.55;
  }

  public FieldPosition getRobotPosition () {
    return new FieldPosition(0,0); //TODO
  }

  public void enableTurningControl(double angle, double tolerance) {
    double startAngle = this.getyaw();
    double temp = startAngle + angle;
   // RobotMap.TURN_P = turningController.getP();
   // RobotMap.TURN_D = turningController.getD();
   // RobotMap.TURN_I = turningController.getI();
    temp = temporaryFixDegrees(temp);
    turningController.setSetpoint(temp);
    turningController.enable();
    turningController.setInputRange(-180.0, 180.0);
    turningController.setOutputRange(-1.0,1.0);
    turningController.setAbsoluteTolerance(tolerance);
    turningController.setToleranceBuffer(1);
    turningController.setContinuous(true);
    turningController.setSetpoint(temp);
}

  public double getTurnOutput() {
    return this.turnOutput;
  }

  double temporaryFixDegrees(double input) {
    if (input > 180) {return input - 360;}
    else if (input < -180){return input + 360;}
    else {return input;}
}

}
