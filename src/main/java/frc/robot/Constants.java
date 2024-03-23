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
    public static final double kControllerDeadbandValue = 0.06;
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
    public static final double kTurningMotorGearReduction = (9424d / 203d);
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0; //radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI;
  

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;


    //PID values for turning
    public static final double kTurningP = 1; //0.2 //1
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
    
    public static final int kDrivingMotorCurrentLimit = 60;
    public static final int kTurningMotorCurrentLimit = 20;

  
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
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 9.1144;
    
    

    
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 1; //0.5
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.8; // 0.3
  
    //These are used for slew rate limiting
    public static final double kTeleDriveMaxAccelerationMetersPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecond = 3;
  
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
  }

  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = 12;
    public static final int kRightIntakeMotorID = 11;
    public static final int kBeltMotorID = 13;


    public static final double kStatorCurrentLimit = 40;

    public static final double kDesiredIntakeMotorRPS = -80; //-60
    public static final double kIntakeBeltMotorSpeed = 0.5;
    public static final double kIntakeCurrentThreshold = -1000;


    public static final double kDesiredShootMotorRPS = 100;
    public static final double kShootBeltMotorSpeed = -0.5;

    public static final double kDesiredFeedMotorRPS = 80;
    public static final double kDesiredFeedBeltSpeed = -0.5;

    public static final double kFlywheelsGearRatio = (34d/15d);



    public static class PIDConstants {
      public static final double kIntakeP = 0.15;
      public static final double kIntakeI = 0;
      public static final double kIntakeD = 0;

      //TODO: Find these values
      //dynamic feedforward
      public static final double kIntakeV = 0;
      //static feedforward
      public static final double kIntakeS = 0;

      //Used for PID to determine what speed is close enough in Rotations per Second
      public static final double kIntakeToleranceThreshold = 3;


      public static final double kMotionMagicCruiseVelocity = 0;
      public static final double kMotionMagicAcceleration = 200;
      public static final double kMotionMagicJerk = 2000;


      public static final double kIntakePIDMinOutput = 1;
      public static final double kIntakePIDMaxOutput = 1;

    }
  }



  public static class PivotConstants {
    public static final int kPivotMotor1ID = 9;
    public static final int kPivotMotor2ID = 10;
    public static final double kPivotEncoderRotationToDegrees = 360 / ((50*64*64)/(8*22*24));
    public static final double kPivotGearRatio = 1d/ ((50*64*64)/(8*22*24));

//TODO: change later
    public static final double kHardStopCurrentThreshold = 10000;

    public static final double kStatorCurrentLimit = 60;


    public static final double kIntakeAngle = 198.9;
    public static final double kSubwooferShootAngle = 61+20.9;
    public static final double kFeedAngle = 150+20.9;


    //Used in multiple commands to automatically move the pivot to a desired angle
    public static class PIDConstants {
      //Proportional term was adjusted to be 360 * 0.017 as the internal PID controller now receives error in rotations and not degrees
      public static final double kPivotP = 20; //0,017
      public static final double kPivotI = 1.6;
      public static final double kPivotD = 0;

      //TODO: Find these values
      //dynamic feedforward
      public static final double kPivotV = 0;
      //static feedforward
      public static final double kPivotS = 0.02;

      //Used for PID to determine what rotation is close enough to desired angle
      public static final double kPivotToleranceThreshold = 2;

      public static final double kPivotPIDMaxOutput = 0.75;
      public static final double kPivotPIDMinOutput = -0.75;
      public static final double kMinSetpoint = 0;
      public static final double kMaxSetpoint = 198.9; //178 max

      public static final double kMotionMagicCruiseVelocity = 1000;
      public static final double kMotionMagicAcceleration = 70;
      public static final double kMotionMagicJerk = 700;

    }
  }

  

  //x and y relative to how far from center of the field, with red side on the right
  //x is long side of field from center, y is short side of field from center
  public static class PredefinedLocations{
   
    public static final double nearPodiumX = 2.66; //105 in
    public static final double nearPodiumY = -2.54; //100 in

    public static final double stageX = 3.81; //150 in
    public static final double stageY = 0.8382; //33 in

    public static final double nearAmpX = 3.302; //130 in
    public static final double nearAmpY = 2.54; //100 in

    
    //PID
    public static final double kXPositionAlignP = 0.5;
    public static final double kXPositionAlignI = 0;
    public static final double kXPositionAlignD = 0;
      
    public static final double kYPositionAlignP = 0.5;
    public static final double kYPositionAlignI = 0;
    public static final double kYPositionAlignD = 0;

    public static final double kTurningPositionAlignP = 0.25;
    public static final double kTurningPositionAlignI = 0;
    public static final double kTurningPositionAlignD = 0;
    
    //Setpoint and Tolerance
    public static final double kYPositionAlignSetpoint = 0;
    public static final double kYPositionAlignTolerance = 0.07;
   
    public static final double kXPositionAlignSetpoint = 0;
    public static final double kXPositionAlignTolerance = 0.07;

    public static final double kTurningPositionSetpoint = 0;
    public static final double kTurningPositionTolerance = 0.07;

    //Min and Max Outputs
    public static final double kTurningPositionMinOutput = -0.3;
    public static final double kTurningPositionMaxOutput = 0.3;
   
    public static final double kYPositionAlignMinOutput = -0.3;
    public static final double kYPositionAlignMaxOutput = 0.3;

    public static final double kXPositionAlignMinOutput = -0.3;
    public static final double kXPositionAlignMaxOutput = 0.3;


  }
  



  public static class PathPlannerConstants { //TODO: GET THESE CONSTANTS
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(0.1, 0, 0);

    //TODO DETERMINE whether this is desired max or physical max
    public static final double MAX_TRANSLATION_SPEED = ChassisConstants.kPhysicalMaxSpeedMetersPerSecond;
    public static final double MAX_ROTATION_SPEED = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    public static final double MAX_TRANSLATION_ACCELERATION = ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond;
    public static final double MAX_ROTATIONAL_ACCELERATION = ChassisConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecond;
    //This constant represents the distance from the origin(center) of your robot to the center of your modules
    //You can plug in the same translation you use when constructing the kinematics object
    public static final double ROBOT_BASE_RADIUS = new Translation2d(ChassisConstants.kWheelBase/2, ChassisConstants.kTrackWidth/2).getNorm();
    
  }

  public static class CommandConstants {

    //These PID values are for commands that line up with the note in the y axis (left/right)
    public static final double kYNoteAlignP = 0.5;
    public static final double kYNoteAlignI = 0;
    public static final double kYNoteAlignD = 0;
    public static final double kYNoteAlignMinOutput = -0.3;
    public static final double kYNoteAlignMaxOutput = 0.3;
    //Meters
    public static final double kYNoteAlignSetpoint = 0;
    public static final double kYNoteAlignTolerance = 0.05;


    public static final double kTurningNoteAlignP = 0.25;
    public static final double kTurningNoteAlignI = 0;
    public static final double kTurningNoteAlignD = 0;
    public static final double kTurningNoteMinOutput = -0.5;
    public static final double kTurningNoteMaxOutput = 0.5;
    //Degrees
    public static final double kTurningNoteAlignSetpoint = 0;
    public static final double kTurningNoteAlignTolerance = 1.5;


    public static final double kXNoteAlignP = 0.3;
    public static final double kXNoteAlignI = 0;
    public static final double kXNoteAlignD = 0;
    public static final double kXNoteAlignMinOutput = -0.3;
    public static final double kXNoteAlignMaxOutput = 0.3;
    //Meters
    public static final double kXNoteAlignSetpoint = 1;
    public static final double kXNoteAlignTolerance = 0.04;
  }

  public static class LimelightConstants {

    public static final double noteHeight = 0; //literally it is 0 
    public static final double noteLimelightMountHeight = 0.668;  //TODO: find limelight mounts
    public static final double noteLimelightMountAngle = -26;

    public static final double tagLimelightMountAngle = 0;
    public static final double tagLimelightMountHeight = 0;

    public static final double[] ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT = {-8.27, -4.1021, 0}; //Meters

    //limelight camera is being weird by 5 degrees
    public static final double kLimelightTXOffset = -5;
  }


  public static class LEDConstants {
    public static final int kBlinkinPort = 0;
  }
}
