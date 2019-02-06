/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static int ct_left_1 = 6;
  public static int ct_left_2 = 7;
  public static int ct_right_1 = 4;
  public static int ct_right_2 = 1;
  public static int ct_center_1 = 5;
  public static int ct_center_2 = 2;

  public static int pcm_id = 9;
  public static final double YAW_ERROR_THRESHOLD = 0.5;

  public static final double ELEVATOR_ENCODER_TICKS_PER_REV = 1024; //Might be different. Change when u find out

  public static final double AUTO_TURN_ACCURACY_THRESHOLD = 3; //in degrees

  public static final double AUTO_DRIVE_DISTANCE_THRESHOLD = 36; //in inches

  public static final double SIDE_GEAR_RATIO = 12 / 72 * 44 / 72;

  public static final double CENTER_GEAR_RATIO = 1 / 12; //Not actually. Fix later

  public static final double SIDE_WHEEL_DIAMETER = 6.0;

  public static final double CENTER_WHEEL_DIAMETER = 4.0;

  public static final double AUTO_ANGLE_DIFFERENTIAL_THRESHOLD = 10; // threshold of difference between angle 1 and angle 2
}
