// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class AutoAlignWithNoteSwerve extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController turningController;
  private PIDController xController;

  private boolean noteIsDetected;
  private double noteYOffset;
  private double noteThetaOffset;
  private double noteXOffset;
  private final SwerveChassisSubsystem swerveSubsystem;
  private final Limelight noteLimelight;
  private final OI oi;



  public AutoAlignWithNoteSwerve(
    SwerveChassisSubsystem swerveSubsystem,
    OI oi,
    Limelight noteLimelight) {
    
      yController = new PIDController(CommandConstants.kYNoteAlignP, CommandConstants.kYNoteAlignI, CommandConstants.kYNoteAlignD);
      yController.setSetpoint(CommandConstants.kYNoteAlignSetpoint);
      yController.setTolerance(CommandConstants.kYNoteAlignTolerance);
  
      turningController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
      turningController.setSetpoint(CommandConstants.kTurningNoteAlignSetpoint);
      turningController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);

      xController = new PIDController(CommandConstants.kXNoteAlignP, CommandConstants.kXNoteAlignI, CommandConstants.kXNoteAlignD);
      xController.setSetpoint(CommandConstants.kXNoteAlignSetpoint);
      xController.setTolerance(CommandConstants.kXNoteAlignTolerance);


    this.swerveSubsystem = swerveSubsystem;
    this.noteLimelight = noteLimelight;
    this.oi = oi;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.enableRobotOriented();
  }

  @Override
  public void execute() {
    double xSpd = oi.xSpeed();
    double ySpd = oi.ySpeed();
    double turningSpd = oi.rotate();

    xSpd = HelperMethods.applyInputDeadband(xSpd);
    ySpd = HelperMethods.applyInputDeadband(ySpd);
    turningSpd = HelperMethods.applyInputDeadband(turningSpd);


    noteIsDetected = noteLimelight.targetDetected();
    SmartDashboard.putBoolean("Note detected", noteIsDetected);


    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));



    //If a note is detected, turning will be disabled and y movement will be controlled by PID
    if (noteIsDetected) {
      noteYOffset = noteLimelight.getXDisplacementFromNote();
      noteThetaOffset = noteLimelight.getTX();
      noteXOffset = noteLimelight.getYDisplacementFromNote();

      SmartDashboard.putNumber("L/R note displacement", noteYOffset);
      SmartDashboard.putNumber("Angle note offset", noteThetaOffset);
      SmartDashboard.putNumber("F/B note displacement", noteXOffset);

      //Field oriented mucks up this command so we disable it when a note is detected
      swerveSubsystem.enableRobotOriented();

      //Calculate pid input using lr offset from note and limit it to max speed values to avoid overshooting
      double PIDySpeed = yController.calculate(noteYOffset);
      PIDySpeed = HelperMethods.limitValInRange(CommandConstants.kYNoteAlignMinOutput, CommandConstants.kYNoteAlignMaxOutput, PIDySpeed);
      double PIDxSpeed = xController.calculate(noteXOffset);
      PIDxSpeed = HelperMethods.limitValInRange(CommandConstants.kXNoteAlignMinOutput, CommandConstants.kXNoteAlignMaxOutput, PIDxSpeed);
     
      swerveSubsystem.driveSwerveWithPhysicalMax(-PIDxSpeed, PIDySpeed, 0);
    }

    //If no note is detected, drive like normal
    else {
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
    System.out.println(xController.atSetpoint());
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
