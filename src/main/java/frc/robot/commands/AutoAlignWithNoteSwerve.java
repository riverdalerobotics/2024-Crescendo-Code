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
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class AutoAlignWithNoteSwerve extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController turningController;

  private boolean noteIsDetected;
  private double noteYOffset;
  private double noteThetaOffset;
  private final SwerveChassisSubsystem swerveSubsystem;
  private final Limelight noteLimelight;

  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;




  public AutoAlignWithNoteSwerve(
    SwerveChassisSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction, 
    Limelight noteLimelight) {
    
      yController = new PIDController(CommandConstants.kYNoteAlignP, CommandConstants.kYNoteAlignI, CommandConstants.kYNoteAlignD);
      yController.setSetpoint(CommandConstants.kYNoteAlignSetpoint);
      yController.setTolerance(CommandConstants.kYNoteAlignTolerance);
  
      turningController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
      turningController.setSetpoint(CommandConstants.kTurningNoteAlignSetpoint);
      turningController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);

    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

    this.swerveSubsystem = swerveSubsystem;
    this.noteLimelight = noteLimelight;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turningSpd = turningSpdFunction.get();


    noteIsDetected = noteLimelight.targetDetected();
    SmartDashboard.putBoolean("Note detected", noteIsDetected);

    if (fieldOrientedFunction.get() && noteIsDetected == false) {
      swerveSubsystem.toggleFieldOriented();
    }



    //If a note is detected, turning will be disabled and y movement will be controlled by PID
    if (noteIsDetected) {
      noteYOffset = noteLimelight.getXDisplacementFromNote();
      noteThetaOffset = noteLimelight.getTX();

      SmartDashboard.putNumber("L/R note displacemnet", noteYOffset);
      SmartDashboard.putNumber("Angle note offset", noteThetaOffset);

      //Field oriented mucks up this command so we disable it when a note is detected
      swerveSubsystem.disableFieldOriented();

      //Calculate pid input using lr offset from note and limit it to max speed values to avoid overshooting
      double PIDySpeed = yController.calculate(noteYOffset);
      PIDySpeed = HelperMethods.limitValInRange(CommandConstants.kYNoteAlignMinOutput, CommandConstants.kYNoteAlignMaxOutput, PIDySpeed);
      //double PIDTurningSpeed = turningController.calculate(noteThetaOffset);
      swerveSubsystem.driveSwerve(xSpd, PIDySpeed, 0);
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
