// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
public class IntakeIndefinitelyCommand extends Command {
  PIDController pivotController;
  PIDController intakeSpeedController;
  double kp = PivotConstants.PIDConstants.kPivotP;
  double ki = PivotConstants.PIDConstants.kPivotI;
  double kd = PivotConstants.PIDConstants.kPivotD;
  double pivotSetpoint = PivotConstants.kIntakeAngle;
  double pivotTolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double intakeSpeed = IntakeConstants.kDesiredIntakeMotorRPS;
  double beltSpeed = IntakeConstants.kIntakeBeltMotorSpeed;
  double maxVoltage = 0d;
  boolean hasPickedUp = false;
  IntakeSubsystem intake;
  PivotSubsystem pivot;
  

  /** Creates a new AutoPickUpCommand. */
  public IntakeIndefinitelyCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.intake = intake;
    addRequirements(pivot, intake);
    pivotController = new PIDController(kp, ki, kd);
    intakeSpeedController = new PIDController(IntakeConstants.PIDConstants.kIntakeP, IntakeConstants.PIDConstants.kIntakeI, IntakeConstants.PIDConstants.kIntakeD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotController.setSetpoint(pivotSetpoint);
    pivotController.setTolerance(pivotTolerance);
    intakeSpeedController.setSetpoint(IntakeConstants.kDesiredIntakeMotorRPS);
    intakeSpeedController.setTolerance(IntakeConstants.PIDConstants.kIntakeToleranceThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.movePivot(pivotController.calculate(pivot.getEncoders()));
    intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
    intake.spinBelt(beltSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotController.reset();
    intakeSpeedController.reset();
    intake.spinBelt(0);
    intake.spinIntake(0);
    pivot.movePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}