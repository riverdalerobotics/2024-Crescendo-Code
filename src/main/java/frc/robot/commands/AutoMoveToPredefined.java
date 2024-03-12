// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.PredefinedLocations;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class AutoMoveToPredefined extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController turningController;
  private PIDController xController;

  private boolean noteIsDetected;
  private double noteYOffset;
  private double noteThetaOffset;
  private double noteXOffset;
  private final SwerveChassisSubsystem swerveSubsystem;
  private final Limelight tagLimelight;
  private final OI oi;

  private double nearPodiumX;
  private double stageX;
  private double nearAmpX;
  private double nearPodiumY = PredefinedLocations.nearPodiumY;
  private double stageY = PredefinedLocations.nearPodiumY;
  private double nearAmpY = PredefinedLocations.nearPodiumX;

  private double xPredefined;
  private double yPredefined;
  
  private double xDisplacementFromRobot;
  private double yDisplacementFromRobot;


  public AutoMoveToPredefined(
    SwerveChassisSubsystem swerveSubsystem,
    OI oi,
    Limelight tagLimelight) {
    
      yController = new PIDController(PredefinedLocations.kYPositionAlignP, PredefinedLocations.kYPositionAlignI, PredefinedLocations.kYPositionAlignD);
      yController.setSetpoint(PredefinedLocations.kYPositionAlignSetpoint);
      yController.setTolerance(PredefinedLocations.kYPositionAlignTolerance);
  
      turningController = new PIDController(PredefinedLocations.kTurningPositionAlignP, PredefinedLocations.kTurningPositionAlignI, PredefinedLocations.kTurningPositionAlignD);
      turningController.setSetpoint(PredefinedLocations.kTurningPositionSetpoint);
      turningController.setTolerance(PredefinedLocations.kTurningPositionTolerance);

      xController = new PIDController(PredefinedLocations.kXPositionAlignP, PredefinedLocations.kXPositionAlignI, PredefinedLocations.kXPositionAlignD);
      xController.setSetpoint(PredefinedLocations.kXPositionAlignSetpoint);
      xController.setTolerance(PredefinedLocations.kXPositionAlignTolerance);


    this.swerveSubsystem = swerveSubsystem;
    this.oi = oi;
    this.tagLimelight = tagLimelight;

    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    swerveSubsystem.enableRobotOriented();
      var alliance = DriverStation.getAlliance();
      if (alliance.get()==DriverStation.Alliance.Blue) { //make x direction reverse if on blue alliance
        nearPodiumX = -Constants.PredefinedLocations.nearPodiumX;
        nearAmpX = -Constants.PredefinedLocations.nearAmpX;
        stageX = -Constants.PredefinedLocations.stageX;
      } else {                                         //make x directions relative to x which it already is set to in constants
        nearPodiumX = Constants.PredefinedLocations.nearPodiumX;
        nearAmpX = Constants.PredefinedLocations.nearAmpX;
        stageX = -Constants.PredefinedLocations.stageX;
      }
  }


  @Override
  public void execute() {
    //System.out.println(xController.atSetpoint());
    double xSpd = oi.xSpeed();
    double ySpd = oi.ySpeed();
    double turningSpd = oi.rotate();

    xSpd = HelperMethods.applyInputDeadband(xSpd);
    ySpd = HelperMethods.applyInputDeadband(ySpd);
    turningSpd = HelperMethods.applyInputDeadband(turningSpd);


    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));

    //calculate vector in absolute to find closest predefined position
    double podiumPredefinedPositionVectorFromRobot = Math.sqrt(Math.pow(nearPodiumY - tagLimelight.getYPosition(), 2) + Math.pow(nearPodiumX - tagLimelight.getXPosition(), 2));
    double ampPredefinedPositionVectorFromRobot = Math.sqrt(Math.pow(nearAmpY - tagLimelight.getYPosition(), 2) + Math.pow(nearAmpX - tagLimelight.getXPosition(), 2));
    double stagePredefinedPositionVectorFromRobot = Math.sqrt(Math.pow(stageY - tagLimelight.getYPosition(), 2) + Math.pow(stageX- tagLimelight.getXPosition(), 2));

    //finds the closest predefined
    if (podiumPredefinedPositionVectorFromRobot >= ampPredefinedPositionVectorFromRobot){
      if(podiumPredefinedPositionVectorFromRobot >= stagePredefinedPositionVectorFromRobot){
        xPredefined = nearPodiumX;
        yPredefined= nearPodiumY;
      } else {
        xPredefined = stageX;
        yPredefined = stageY;
      }
    } else{
      if (ampPredefinedPositionVectorFromRobot >= stagePredefinedPositionVectorFromRobot){
        xPredefined = nearAmpX;
        yPredefined = nearAmpY;
      } else {
        xPredefined = stageX;
        yPredefined = stageY;
      }

      //finds x and y displacement from predefined
      xDisplacementFromRobot = xPredefined - tagLimelight.getXPosition();
      yDisplacementFromRobot = yPredefined - tagLimelight.getYPosition();

    //If a tag  is detected, x and y movement will be controlled by PID
    if (tagLimelight.targetDetected() == true) {
    //finds x and y displacement from predefined


      SmartDashboard.putNumber("XDisplacementNeededToTravel", xDisplacementFromRobot);
      SmartDashboard.putNumber("YDisplacementNeededToTravel", yDisplacementFromRobot);

      //Field oriented mucks up this command so we disable it when a note is detected
      swerveSubsystem.enableRobotOriented();

      //Calculate pid input using lr offset from note and limit it to max speed values to avoid overshooting
      double PIDySpeed = yController.calculate(yDisplacementFromRobot);
      PIDySpeed = HelperMethods.limitValInRange(PredefinedLocations.kYPositionAlignMinOutput, CommandConstants.kYNoteAlignMaxOutput, PIDySpeed);
      double PIDxSpeed = xController.calculate(xDisplacementFromRobot);
      PIDxSpeed = HelperMethods.limitValInRange(CommandConstants.kXNoteAlignMinOutput, CommandConstants.kXNoteAlignMaxOutput, PIDxSpeed);
     
      swerveSubsystem.driveSwerveWithPhysicalMax(PIDxSpeed, PIDySpeed, 0);
      
//      SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    }
    
    
    //If no note is detected, drive like normal
    else {
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
    SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    System.out.println(xController.atSetpoint());
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    yController.reset();
    turningController.reset();
    xController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint()) {
      return true;
    }
    else {
      return false;
    }
  }
}
