// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
public class ShootCommand extends Command {
  PIDController pivotSpeedController;
  PIDController shootSpeedContller;
  //Pivot pid constants
  double Pkp = PivotConstants.PIDConstants.kPivotP;
  double Pki = PivotConstants.PIDConstants.kPivotI;
  double Pkd = PivotConstants.PIDConstants.kPivotD;
  //shooter pid constants
  double Skp = IntakeConstants.PIDConstants.kIntakeP;
  double Ski = IntakeConstants.PIDConstants.kIntakeI;
  double Skd = IntakeConstants.PIDConstants.kIntakeD;
  //other stuff
  double desiredPivotAngle;
  double shootTolerance = IntakeConstants.PIDConstants.kIntakeToleranceThreshold;
  double pivotTolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double intakeSpeed = IntakeConstants.kDesiredShootMotorRPS;
  double beltSpeed = IntakeConstants.kShootBeltMotorSpeed;
  double startTime = 0d;
  double currentTime = 0d;
  double shootTime = 0d;
  double timeNeeded = 0d;
  boolean beltEngaged = false;
  boolean hasShot = false;
  OI oi;

  private PivotSubsystem pivot;
  private IntakeSubsystem intake;

  /** Creates a new AutoPickUpCommand. */
  public ShootCommand(OI oi, PivotSubsystem pivot, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, pivot);
    pivotSpeedController = new PIDController(Pkp, Pki, Pkd);
    shootSpeedContller = new PIDController(Skp, Ski, Skd);
    this.desiredPivotAngle = PivotConstants.kSubwooferShootAngle;
    this.oi = oi;
    this.pivot = pivot;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSpeedController.setSetpoint(desiredPivotAngle);
    pivotSpeedController.setTolerance(pivotTolerance);
    shootSpeedContller.setSetpoint(intakeSpeed);
    shootSpeedContller.setTolerance(shootTolerance);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //TODO: Pivot and and Shoot should both go at same time. When both at setpoint, belt note in
  public void execute() {
    currentTime = System.currentTimeMillis();
    pivot.movePivot(pivotSpeedController.calculate(pivot.getEncoders()));
    intake.spinIntake(shootSpeedContller.calculate(intake.getSpeed()));
 
      
    if (shootSpeedContller.atSetpoint() && shootSpeedContller.atSetpoint()){
        if (oi.shoot()){
          intake.spinBelt(beltSpeed);
          startTime =  System.currentTimeMillis();
          shootTime = startTime + timeNeeded;
          beltEngaged = true;
        }
        if (currentTime > shootTime && beltEngaged){
            intake.spinBelt(0);
            intake.spinIntake(0);
            hasShot = true;
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSpeedController.reset();
    shootSpeedContller.reset();
    pivot.movePivot(0);
    intake.spinBelt(0);
    intake.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSpeedController.atSetpoint() && shootSpeedContller.atSetpoint() && hasShot && beltEngaged;
  }
}
