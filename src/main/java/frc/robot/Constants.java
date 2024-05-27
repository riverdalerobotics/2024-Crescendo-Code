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


  //Liam comment!!!!!!
  //Constants for you to create
  //intakeMotorCurrent limit (40)
  //flywheelsGearRatio (34t gear being driven by a 15t gear)
  //fill out some basic PID constants
  public static class IntakeConstants {
    public static final int kLeftIntakeMotorID = 12;
    public static final int kRightIntakeMotorID = 11;
    public static final int kBeltMotorID = 13;


    public static final double kStatorCurrentLimit = 40;

    public static final double kFlywheelsGearRatio = (34d/15d);
    public static final double peakDutyCycle = 1;

    public static class PIDConstants {
      public static final double kIntakeP = -5834;
      public static final double kIntakeI = -5834;
      public static final double kIntakeD = -5834;

     
      //Your dynamic feedforward value has already been defined for you because we are so nice :D
      public static final double kIntakeV = 0.057;
      
      
      //static FF. We should use this but our intake has worked very well without it so we dont mess with it 
      public static final double kIntakeS = 0;

      //Used for PID to determine what speed is close enough in Rotations per Second
      public static final double kIntakeToleranceThreshold = -5834;


    }
  }

  public static class LEDConstants {
    public static final int kBlinkinPort = 0;
  }
}
