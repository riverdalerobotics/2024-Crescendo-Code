package frc.robot.commands.intakeCommands;



// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.OI;
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
  OI oi;
  double beltSpeed;
  boolean manualBeltControl;
  public NewAutoRevFlywheelsIndefinitely(double speedRPS, double beltSpeed, IntakeSubsystem intakeSubsystem, BlinkinLED LED, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeSubsystem;
    this.desiredSpeedRPS = speedRPS;
    this.LED = LED;
    this.oi = oi;
    this.beltSpeed = beltSpeed;
    if (beltSpeed == 0) {
      manualBeltControl = true;
    }
    else{
      manualBeltControl = false;
    }
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
    intake.specCommandRunning = true;
    if (intake.getLeftIntakeMotor().atSetpointPosition(desiredSpeedRPS)) {
      LED.disableFlywheelsRevvingLED();
      LED.enableFlywheelsReadyLED();
    }
    if (manualBeltControl) {
      //During operation, the driver can hold down the right bumper to power the indexer to shoot
      if(oi.shoot()) {
        intake.spinBelt(IntakeConstants.kShootBeltMotorSpeed);
      }
      else {
        intake.spinBelt(0);
      }
    }
    else {
      intake.spinBelt(beltSpeed);
    }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  intake.setIntakeVelocityRPS(0);
  intake.spinBelt(0);
  LED.disableFlywheelsRevvingLED();
  LED.disableFlywheelsReadyLED();
  }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
 }
