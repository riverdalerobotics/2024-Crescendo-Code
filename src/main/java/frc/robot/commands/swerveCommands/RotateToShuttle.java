// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;


import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class RotateToShuttle extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController turningController;

  private boolean noteIsDetected;
  private final SwerveChassisSubsystem swerveSubsystem;
  
  private final OI oi;
  private final BlinkinLED LED;
  private double setpoint;
  Optional<Alliance> ally = DriverStation.getAlliance();


  //TODO: Consider using slew rate limiters here
  public RotateToShuttle(
    SwerveChassisSubsystem swerveSubsystem,
    OI oi,
    BlinkinLED LED) {

    if (ally.get() == Alliance.Red) {
        setpoint = 145;  
    }
    if (ally.get() == Alliance.Blue) {
         setpoint = -145;
    }
      
    
    turningController = new PIDController(CommandConstants.kTurningShuttleP, CommandConstants.kTurningShuttleI, CommandConstants.kTurningShuttleD);
    //turningController.enableContinuousInput(-180, 180);
    turningController.setSetpoint(setpoint);
    turningController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);

    this.swerveSubsystem = swerveSubsystem;
    this.LED = LED;
    this.oi = oi;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.enableFieldOriented();
    swerveSubsystem.setDrivesBrake();
  }

  @Override
  public void execute() {

    double xSpd = oi.xSpeed();
    double ySpd = oi.ySpeed();
    double turningSpd = oi.rotate();

    xSpd = HelperMethods.applyInputDeadband(xSpd);
    ySpd = HelperMethods.applyInputDeadband(ySpd);
    turningSpd = HelperMethods.applyInputDeadband(turningSpd);

    
    SmartDashboard.putBoolean("Note detected", noteIsDetected);

    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));



    //if b button is presed, rotate robot to shuttle angle, 
    if (oi.rotateRobot() == true) {
      swerveSubsystem.setDrivesBrake();
      double offset = -swerveSubsystem.getHeadingDegrees();

      double PIDThetaSpeed = turningController.calculate(offset);
      PIDThetaSpeed = HelperMethods.limitValInRange(CommandConstants.kTurningNoteMinOutput, CommandConstants.kTurningNoteMaxOutput, PIDThetaSpeed);
      
      swerveSubsystem.driveSwerve(xSpd, ySpd, PIDThetaSpeed);
    }
    else{
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   
    turningController.reset();

    swerveSubsystem.stopModules();
    swerveSubsystem.setDrivesBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (turningController.atSetpoint()) {
      return true;
    }
    else {
      return false;
    }
  }
}
