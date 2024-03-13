// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.





//The purpose of this command is to be called after the shooting wheels have already been revved up



package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class PowerBeltAndShooter extends Command {
  /** Creates a new PowerBeltAndShooter. */
  PIDController intakeSpeedController;
  IntakeSubsystem intake;
  double kp = IntakeConstants.PIDConstants.kIntakeP;
  double ki = IntakeConstants.PIDConstants.kIntakeI;
  double kd = IntakeConstants.PIDConstants.kIntakeD;
  double tolerance = IntakeConstants.PIDConstants.kIntakeToleranceThreshold;
  double desiredSpeed = IntakeConstants.kDesiredShootMotorRPS;
  double beltSpeed = IntakeConstants.kShootBeltMotorSpeed;
  boolean hasFired = false;

  //TODO: make a constant for this
  double maxCurrent = 1000;
  BlinkinLED LED;
  public PowerBeltAndShooter(IntakeSubsystem intakeSubsystem, BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeSubsystem;
    intakeSpeedController = new PIDController(kp, ki, kd);
    this.LED = LED;
    addRequirements(intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Sets the desired speed of the fly wheels in the PID controller
    intakeSpeedController.setSetpoint(desiredSpeed);

    //Sets the tolerance the controller considers to be close enough to the desired speed
    //(This doesnt actually do anything in this command)
    intakeSpeedController.setTolerance(tolerance);
    LED.enableFlywheelsReadyLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Spins the belt and fly wheels
    intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
    intake.spinBelt(beltSpeed);


    //TODO: probably should make a short delay for this to end the command
    //TODO: we could also just changed this and make it timed. Would be more consistent and basically 
    //do it in the same time
    //This detects when the note has been shot out of the fly wheels, ending the command
    if(intake.intakeCurrent() > maxCurrent){
      intake.spinBelt(0);
      intake.spinIntake(0);
      hasFired = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //The motor speeds should have already been stopped at this point, but good to be safe
    intakeSpeedController.reset();
    intake.spinBelt(0);
    intake.spinIntake(0);
    LED.disableFlywheelsReadyLED();
    LED.disableFlywheelsRevvingLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasFired;
  }
}
