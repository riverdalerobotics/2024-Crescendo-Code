// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRevFlyWheels extends Command {
  /** Creates a new AutoRevFlyWheels. */
  PIDController intakeSpeedController;
  IntakeSubsystem intake;
  double kp = IntakeConstants.PIDConstants.kIntakeP;
  double ki = IntakeConstants.PIDConstants.kIntakeI;
  double kd = IntakeConstants.PIDConstants.kIntakeD;
  double tolerance = IntakeConstants.PIDConstants.kIntakeToleranceThreshold;
  double desiredSpeedRPS;
  BlinkinLED LED;
  public AutoRevFlyWheels(double speedRPS, IntakeSubsystem intakeSubsystem, BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSpeedController = new PIDController(kp, ki, kd);
    this.intake = intakeSubsystem;
    this.desiredSpeedRPS = speedRPS;
    this.LED = LED;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeedController.setSetpoint(desiredSpeedRPS);
    intakeSpeedController.setTolerance(tolerance);
    LED.enableFlywheelsRevvingLED();
  }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
//   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  intakeSpeedController.reset();
  LED.disableFlywheelsRevvingLED();
  }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return intakeSpeedController.atSetpoint();
//   }
// }
