// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = Units.lbsToKilograms(108); /// 108 Pounds as of 3/3/25
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(10); // was 14.5
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0); // 5.5
    public static final PIDConstants ANGLE_PID       = new PIDConstants(5.5, 0, 0);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class VisionConstants
  {
    // https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Fwww.best-microcontroller-projects.com%2Fimage-files%2Faircraft-roll-pitch-yaw-xyz-enh.jpg&f=1&nofb=1&ipt=ac25795fcd02d469477e44fc513971b36be4609b831093f83e534bd2a382961d&ipo=images
    public static final String MAIN_CAM_NAME = "HD_USB_Camera";
    public static final Rotation3d MAIN_CAM_ROTATION = new Rotation3d(
      0,  // Roll
      Units.degreesToRadians(30), // Units.degreesToRadians(90), // Pitch before was 0
      0 // Yaw
    );
    public static final Translation3d MAIN_CAM_TRANSLATION = new Translation3d( // Meters
      0.31, // X
      -0.18,  // Y
      0.98   // Z
    );
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ElevatorConstants
  {
    public static final int LEADER = 20;
    public static final int FOLLOWER = 21;

    // public static final double POS_ZERO = 1;
    public static final double POS_ONE = 0;
    public static final double POS_TWO = 4;
    public static final double POS_THREE = 13;
    public static final double POS_FOUR = 26.2;
    public static final double POS_DEFAULT = POS_ONE;

    public static final double TOLERANCE = 2;
    public static final PIDConstants PID_CONSTANTS = new PIDConstants(0.2,0.0,0.5);

    public static final double MAX_SPEED = 0.2;
    public static final double MIN_SPEED = -0.1;
  }

  public static class ArmConstants
  {
    public static final int PIVOT = 30;

    public static final double POS_UP = -34;
    public static final double POS_DOWN = -10;
    public static final double POS_DEFAULT = POS_UP;

    public static final double TOLERANCE = 4;
    public static final PIDConstants PID_CONSTANTS = new PIDConstants(0.03,0,0.03,0);

    public static final double MAX_SPEED = 0.1;
    public static final double MIN_SPEED = -0.1;
  } 

  public static class CoralIntakeConstants
  {
    public static final int LEADER = 22;
    public static final int FOLLOWER = 23;
    public static final int LASER_CAN = 24;

    public static final double THRESHOLD_MM = 12;

    public static final double HIGH_SPEED = 0.22; // 0.3
    public static final double SLOW_SPEED = 0.1; 
  }

  public static class AlgaeIntakeConstants
  {
    public static final int LEADER = 31;
    public static final int FOLLOWER = 32;  

    public static final double SPEED = 0.5; // TODO
  }
}
