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

  public static class PathPlannerConstants{ //TODO: GET THESE CONSTANTS
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(-1, -1, -1);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(-1, -1, -1);
    public static final double MAX_TRANSLATION_SPEED = kTeleDriveMaxSpeedMetersPerSecond;
    public static final double MAX_ROTATION_SPEED = kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double MAX_TRANSLATION_ACCELERATION = kTeleDriveMaxAccelerationMetersPerSecond;
    public static final double MAX_ROTATIONAL_ACCELERATION = kTeleDriveMaxAngularAccelerationRadiansPerSecond;
    public static final double ROBOT_BASE_RADIUS = -1; // meters
  
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
    //The native unit is rotations. We conver it to meters
    public static final double kDrivingEncoderPositionFactor = (kWheelCircumferenceMeters / kDriveMotorGearReduction);

    //in meters
    //The native units is RPM. We convert it to m/s
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60; //meters per second


    //One full rotation of the turning motor is 2pi radians
    public static final double kTurningMotorGearReduction = (9424 / 203);
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / (kTurningMotorGearReduction); // radians
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; //radians per second

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
    //TODO: Test both of these 
    public static final double kPhysicalMaxSpeedMetersPerSecond = 14.9;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 1;
    
    

    
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 1;
  
    //These are used for slew rate limiting
    public static final double kTeleDriveMaxAccelerationMetersPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecond = 3;
  
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
  }



    public static class PathPlannerConstants { //TODO: GET THESE CONSTANTS
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(-1, -1, -1);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(-1, -1, -1);
    public static final double MAX_TRANSLATION_SPEED = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double MAX_ROTATION_SPEED = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double MAX_TRANSLATION_ACCELERATION = ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond;
    public static final double MAX_ROTATIONAL_ACCELERATION = ChassisConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecond;
    public static final double ROBOT_BASE_RADIUS = -1; // meters
  
  }

  public static class CommandConstants {

    //These PID values are for commands that line up with the note in the y axis (left/right)
    public static final double kYNoteAlignP = 0.1;
    public static final double kYNoteAlignI = 0;
    public static final double kYNoteAlignD = 0;

    public static final double kTurningNoteAlignP = 0.1;
    public static final double kTurningNoteAlignI = 0;
    public static final double kTurningNoteAlignD = 0;

    public static final double kXNoteAlignP = 0.1;
    public static final double kXNoteAlignI = 0;
    public static final double kXNoteAlignD = 0;
  }

  public static class LimelightConstants {

    public static final double noteHeight = 0; //literally it is 0 
    public static final double noteLimelightMountAngle = 0;  //TODO: find limelight mounts
    public static final double noteLimelightMountHeight = 0;

    public static final double tagLimelightMountAngle = 0;
    public static final double tagLimelightMountHeight = 0;

    public static final double[] ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT = {-827, -4.1021, 0} //Meters
  }


  public static class LEDConstants {
    public static final int kBlinkinPort = -1;
  }
}
