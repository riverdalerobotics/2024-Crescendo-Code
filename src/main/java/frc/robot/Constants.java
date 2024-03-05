// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
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
    public static final int kOperatorControllerPort = 1;
  }

  
  public static class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;


    //public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = 0.0762; 

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



    //TODO: Test all these conversion FACTORS
    //One full rotation of the turning motor is 2pi radians
    public static final double kTurningMotorGearReduction = (9424 / 203);
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; //radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI;
  

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;


    //PID values for turning
    public static final double kTurningP = 1; //0.2
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;
    //5 degrees in radians
    public static final double kTurningTolerance = 0.0872665;


    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveFF = 0;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;




    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

  
  }

  public static class ChassisConstants {


    //Both of these variables are in meters

    //distance between the centers of the left and right modules
    public static final double kTrackWidth = 0.54587;
    //distance between the centers of the front and back modules
    public static final double kWheelBase = 0.59667;


    //Full chassis dimensions
    public static final double chassisWidth = Units.inchesToMeters(25);
    public static final double chassisLength = Units.inchesToMeters(27);

    //Note for translation 2d. Positive x is forward, and positive y is left
    //The order of module position construction in our kinematics object is as follows
    //FrontLeft, FrontRight, BackLeft, BackRight
    //This order must be followed when passing in and receiving module states from the kinematics object
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));



  
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 8;



    //TODO: Test these
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;



    //These are the absolute max limitations of the robot
    //TODO: Test both of these 
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 9.1144;
    
    

    
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; //0.5
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 1; // 0.3
  
    //These are used for slew rate limiting
    public static final double kTeleDriveMaxAccelerationMetersPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecond = 3;
  
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
  }

  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = -1000;
    public static final int kRightIntakeMotorID = -1000;
    public static final int kBeltMotorID = -1000;
    public static final int kSpeedEncoderID = -1000;



    public static final double kDesiredIntakeMotorRPS = -1000;
    public static final double kIntakeBeltMotorSpeed = -1000;


    public static final double kDesiredShootMotorRPS = -1000;
    public static final double kShootBeltMotorSpeed = -1000;



    public static class PIDConstants {
      public static final double kIntakeP = -1000;
      public static final double kIntakeI = -1000;
      public static final double kIntakeD = -1000;
      //Used for PID to determine what speed is close enough in Rotations per Second
      public static final double kIntakeToleranceThreshold = -1000;

    }
  }



  public static class PivotConstants {
    public static final int kPivotMotor1ID = -1000;
    public static final int kPivotMotor2ID = -1000;
    public static final int kPivotEncoderID = -1000;
    public static final double kPivotEncoderRotationToDegrees = 18152.7272727;

    public static final int kMinPivotRotationDegrees = 0;
    //TODO: find max rotation value for pivot of shooter
    public static final int kMaxPivotRotationDegrees = -1000;


    //Max voltage pivot can receive before attempting to reset position
    public static final int kPivotMaxVoltage = -1000;


    public static final int kIntakeAngle = -1000;
    public static final int kSubwooferShootAngle = -1000;


    //Used in multiple commands to automatically move the pivot to a desired angle
    public static class PIDConstants {
      public static final double kPivotP = -1000;
      public static final double kPivotI = -1000;
      public static final double kPivotD = -1000;
      //Used for PID to determine what rotation is close enough to desired angle
      public static final double kPivotToleranceThreshold = -1000;

    }

  }





  public static class PathPlannerConstants { //TODO: GET THESE CONSTANTS
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(0.1, 0, 0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(0.1, 0, 0);
    public static final double MAX_TRANSLATION_SPEED = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
    public static final double MAX_ROTATION_SPEED = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double MAX_TRANSLATION_ACCELERATION = ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond;
    public static final double MAX_ROTATIONAL_ACCELERATION = ChassisConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecond;
    //This constant represents the distance from the origin(center) of your robot to the center of your modules
    //You can plug in the same translation you use when constructing the kinematics object
    public static final double ROBOT_BASE_RADIUS = new Translation2d(ChassisConstants.kWheelBase/2, ChassisConstants.kTrackWidth/2).getNorm();
    
  }

  public static class CommandConstants {

    //These PID values are for commands that line up with the note in the y axis (left/right)
    public static final double kYNoteAlignP = 0.01;
    public static final double kYNoteAlignI = 0;
    public static final double kYNoteAlignD = 0;
    public static final double kYNoteAlignMinOutput = -0.2;
    public static final double kYNoteAlignMaxOutput = 0.2;
    //Meters
    public static final double kYNoteAlignSetpoint = 0;
    public static final double kYNoteAlignTolerance = 0.05;


    public static final double kTurningNoteAlignP = 0.1;
    public static final double kTurningNoteAlignI = 0;
    public static final double kTurningNoteAlignD = 0;
    //Degrees
    public static final double kTurningNoteAlignSetpoint = 0;
    public static final double kTurningNoteAlignTolerance = 1;


    public static final double kXNoteAlignP = 0.1;
    public static final double kXNoteAlignI = 0;
    public static final double kXNoteAlignD = 0;
    //Meters
    public static final double kXNoteAlignSetpoint = 0.5;
    public static final double kXNoteAlignTolerance = 0.05;
  }

  public static class LimelightConstants {

    public static final double noteHeight = 0; //literally it is 0 
    public static final double noteLimelightMountAngle = 0.63350946;  //TODO: find limelight mounts
    public static final double noteLimelightMountHeight = 17;

    public static final double tagLimelightMountAngle = 0;
    public static final double tagLimelightMountHeight = 0;

    public static final double[] ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT = {-827, -4.1021, 0}; //Meters
  }


  public static class LEDConstants {
    public static final int kBlinkinPort = -1;
  }
}
