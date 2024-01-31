// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static String APRILTAG_JSON_NAME = "tags.json";
  public static String CAMERA_JSON_NAME = "cams.jsons";
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ControllerConstants {
    public static int kDrivingJoystickPort = 0;
    public static int kHelperJoystickPort = 1;
  }
  public static class DriveConstants {
    public static double kMaxSpeedMetersPerSecond = 4.8;


    // Chassis configuration
    public static double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels
    public static double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      );


    // SPARK MAX CAN IDs
    public static int kFrontLeftDrivingCanId = 11;
    public static int kRearLeftDrivingCanId = 13;
    public static int kFrontRightDrivingCanId = 15;
    public static int kRearRightDrivingCanId = 17;


    public static int kFrontLeftTurningCanId = 10;
    public static int kRearLeftTurningCanId = 12;
    public static int kFrontRightTurningCanId = 14;
    public static int kRearRightTurningCanId = 16;


    public static int kFrontLeftEncoderCanId = 17;
    public static int kRearLeftEncoderCanId = 18;
    public static int kFrontRightEncoderCanId = 19;
    public static int kRearRightEncoderCanId = 20;


    public static boolean kGyroReversed = false;
  }


  public static class ModuleConstants {
    public static double kFreeSpeedRpm = 5676;


    public static double kDrivingMotorPinionTeeth = 14;


    public static boolean kTurningEncoderInverted = true;


    public static double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    public static double kWheelDiamaterMeters = .1016;
    public static double kWheelCircumferenceMeters = kWheelDiamaterMeters * Math.PI;


    public static double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;


    public static double kDrivingEncoderPositionFactor = (kWheelDiamaterMeters * Math.PI) / kDrivingMotorReduction;
    public static double kDrivingEncoderVelocityFactor = ((kWheelDiamaterMeters * Math.PI) / kDrivingMotorReduction) * 60.0;


    public static double kTurningEncoderPositionPIDMinInput = 0.0;
    public static double kTurningEncoderPositionPIDMaxInput = 360.0;


    public static double kDrivingP = 0.04;
    public static double kDrivingI = 0;
    public static double kDrivingD = 0;
    public static double kDrivingFF = 1/kDriveWheelFreeSpeedRps;
    public static double kDrivingMinOutput = -1;
    public static double kDrivingMaxOutput = 1;


    public static double kTurningP = 0.0027;
    public static double kTurningI = 0;
    public static double kTurningD = 0;
    public static double kTurningFF = 0;
    public static double kTurningMinOutput = -1;
    public static double kTurningMaxOutput = 1;


    public static com.revrobotics.CANSparkBase.IdleMode kDrivingMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
    public static com.revrobotics.CANSparkBase.IdleMode kTurningMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;


    public static int kDrivingMotorCurrentLimit = 50; // amps
    public static int kTurningMotorCurrentLimit = 20;
  }
}


