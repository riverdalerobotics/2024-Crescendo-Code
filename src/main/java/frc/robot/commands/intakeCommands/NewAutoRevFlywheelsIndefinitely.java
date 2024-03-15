package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.TalonHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * Revs the arm fly wheels to the desired rotaions per second. This command ends as soon as the desired
 * RPM is reached. The default command doesn't retain this RPS so this command should only be used
 * when the next command is continuing to power the flywheels
 */
public class NewAutoRevFlywheelsIndefinitely extends Command {
  /** Creates a new AutoRevFlyWheels. */
  IntakeSubsystem intake;
  double tolerance = IntakeConstants.PIDConstants.kIntakeToleranceThreshold;
  double desiredSpeedRPS;
  BlinkinLED LED;
  public NewAutoRevFlywheelsIndefinitely(double speedRPS, IntakeSubsystem intakeSubsystem, BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeSubsystem;
    this.desiredSpeedRPS = speedRPS;
    this.LED = LED;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeVelocityRPS(desiredSpeedRPS);
    LED.enableFlywheelsRevvingLED();
    intake.setIntakeTolerance(tolerance);
  }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    if (intake.getLeftIntakeMotor().atSetpointPosition(desiredSpeedRPS)) {
      LED.disableFlywheelsRevvingLED();
      LED.enableFlywheelsReadyLED();
    }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  intake.setIntakeVelocityRPS(0);
  LED.disableFlywheelsRevvingLED();
  LED.disableFlywheelsReadyLED();
  }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
 }
