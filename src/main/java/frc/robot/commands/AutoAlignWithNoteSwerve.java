// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class AutoAlignWithNoteSwerve extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  PIDController yController;
  PIDController turningController;

  boolean noteIsDetected;
  double noteYOffset;
  double noteThetaOffset;
  SwerveChassisSubsystem swerveSubsystem;

  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final Supplier<Boolean> toggleSlowModeFunction;



  public AutoAlignWithNoteSwerve(SwerveChassisSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> toggleSlow) {
    yController = new PIDController(CommandConstants.kYNoteAlignP, CommandConstants.kYNoteAlignI, CommandConstants.kYNoteAlignD);
    yController.setSetpoint(0);

    turningController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
    turningController.setSetpoint(0);


    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.toggleSlowModeFunction = toggleSlow;

    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turningSpd = turningSpdFunction.get();


    if (fieldOrientedFunction.get()) {
      swerveSubsystem.toggleFieldOriented();
    }

    if (toggleSlowModeFunction.get()) {
      //Toggle slow
    }

    //If a note is detected, turning will be disabled and y movement will be controlled by PID
    if (noteIsDetected) {
      //Field oriented mucks up this command so we disable it when a note is detected
      swerveSubsystem.disableFieldOriented();
      double PIDySpeed = yController.calculate(noteYOffset);
      double PIDTurningSpeed = turningController.calculate(noteThetaOffset);
      swerveSubsystem.driveSwerve(xSpd, PIDySpeed, PIDTurningSpeed);
    }

    //If no note is detected, drive like normal
    else {
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
