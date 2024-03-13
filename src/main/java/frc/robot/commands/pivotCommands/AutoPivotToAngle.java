// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.HelperMethods;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotToAngle extends Command {
  /** Creates a new AutoPivotToAngle. */
  double kp = PivotConstants.PIDConstants.kPivotP;
  double ki = PivotConstants.PIDConstants.kPivotI;
  double kd = PivotConstants.PIDConstants.kPivotD;
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double desiredAngle;
  PivotSubsystem pivot;
  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  double hardStopPosition = PivotConstants.PIDConstants.kMinSetpoint;
  PIDController pivotController;
  public AutoPivotToAngle(double angle, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivotController = new PIDController(kp, ki, kd);
    this.pivot = pivotSubsystem;
    this.desiredAngle = angle;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotController.setSetpoint(desiredAngle);
    pivotController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.movePivot(HelperMethods.limitValInRange(PivotConstants.PIDConstants.kPivotPIDMinOutput, PivotConstants.PIDConstants.kPivotPIDMaxOutput, pivotController.calculate(pivot.getEncoders())));
  
    //TODO: Fix the logic for this because it kind of sucks rn
    if (pivot.getCurrent() > maxCurrent){
      pivotController.setSetpoint(hardStopPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotController.reset();
    pivot.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotController.atSetpoint();
  }
}
