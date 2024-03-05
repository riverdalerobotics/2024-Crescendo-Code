// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class PowerBeltAndShooter extends Command {
  /** Creates a new PowerBeltAndShooter. */
  PIDController intakeSpeedController;
  IntakeSubsystem intake;
  double kp = IntakeConstants.PIDConstants.kIntakeP;
  double ki = IntakeConstants.PIDConstants.kIntakeI;
  double kd = IntakeConstants.PIDConstants.kIntakeD;
  double tolerance = 0d;
  double desiredSpeed;
  double beltSpeed;
  boolean hasPickedUp = false;
  double maxCurrent = 0d;
  public PowerBeltAndShooter(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeSubsystem;
    intakeSpeedController = new PIDController(kp, ki, kd);
    addRequirements(intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeedController.setSetpoint(desiredSpeed);
    intakeSpeedController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
    intake.spinBelt(beltSpeed);
    if(intake.intakeCurrent()>maxCurrent){
      intake.spinBelt(0);
      intake.spinIntake(0);
      hasPickedUp = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSpeedController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPickedUp;
  }
}
