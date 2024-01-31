// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;


    //public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = -1000;

    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    //This constant was taken directly from the MAXSwerve example code. 
    //TODO: Confirm this ratio is correct
    //THIS VALUE DOES NOT REPRESENT THE GEAR RATIO. IT is the gear ratio's reciprocal
    //ie if gear ratio was 1:4 this would be 4
    public static final double kDriveMotorGearReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);

    public static final double kDriveWheelFreeSpeedRps = -1000;

    //In meters
    //What 1 rotation of the drive motor corresponds to in meters travelled
    public static final double kDrivingEncoderPositionFactor = (kWheelCircumferenceMeters / kDriveMotorGearReduction);

    public static final double kDrivingEncoderVelocityFactor = (kWheelDiameterMeters / kDriveMotorGearReduction) / 60; //meters per second


    //One full rotation of the turning motor is 2pi radians
    public static final double kTurningEncoderPositionFactor = 2 * Math.PI; // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; //radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
  

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;


    //PID values for turning
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;




    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  
  }

  public static class ChassisConstants {

    //distance between the centers of the left and right modules
    public static final double kTrackWidth = Units.inchesToMeters(25);

    //distance between the centers of the front and back modules
    public static final double kWheelBase = Units.inchesToMeters(27);

    //Note for translation 2d. Positive x is forward, and positive y is left
    //The order of module position construction in our kinematics object is as follows
    //FrontLeft, FrontRight, BackLeft, BackRight
    //This order must be followed when passing in and receiving module states from the kinematics object
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    public static final int kFrontLeftDriveMotorPort = -100; 
    public static final int kBackLeftDriveMotorPort = -100; 
    public static final int kFrontRightDriveMotorPort = -100;
    public static final int kBackRightDriveMotorPort = -100;

    public static final int kFrontLeftTurningMotorPort = -100;
    public static final int kBackLeftTurningMotorPort = -100;
    public static final int kFrontRightTurningMotorPort = -100;
    public static final int kBackRightTurningMotorPort = -100;



    //TODO: Test these
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;



    //These are the absolute max limitations of the robot
    //TODO: Test
    public static final double kPhysicalMaxSpeedMetersPerSecond = 14.9;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = -1000;
    
    

    
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 1;
  
  
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
  }
}
