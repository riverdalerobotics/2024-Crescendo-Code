// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
public class ShootCommand extends Command {
  PIDController pivotSpeedController;
  PIDController shootSpeedContller;
  double Pkp = 0d;
  double Pki = 0d;
  double Pkd = 0d;
  double Skp = 0d;
  double Ski = 0d;
  double Skd = 0d;
  double setpoint = 0d;
  double velocity = 0d;
  double shootTolerance;
  double pivotTolerance = 0d;
  double intakeSpeed = 0d;
  double beltSpeed = 0d;
  double startTime = 0d;
  double currentTime = 0d;
  double shootTime = 0d;
  double timeNeeded = 0d;
  boolean hasReved = false;
  boolean hasShot = false;
  OI oi;

  /** Creates a new AutoPickUpCommand. */
  public ShootCommand(OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.PIVOT);
    addRequirements(RobotContainer.INTAKE);
    pivotSpeedController = new PIDController(Pkp, Pki, Pkd);
    shootSpeedContller = new PIDController(Skp, Ski, Skd);
    this.oi = oi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSpeedController.setSetpoint(setpoint);
    pivotSpeedController.setTolerance(pivotTolerance);
    shootSpeedContller.setSetpoint(intakeSpeed);
    shootSpeedContller.setTolerance(shootTolerance);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  //TODO: Pivot and and Shoot should both go at same time. When both at setpoint, belt note in
  public void execute() {
    currentTime = System.currentTimeMillis();
    RobotContainer.PIVOT.movePivot(pivotSpeedController.calculate(RobotContainer.PIVOT.getEncoders()));
    if(pivotSpeedController.atSetpoint()){
      RobotContainer.INTAKE.spinIntake(shootSpeedContller.calculate(RobotContainer.INTAKE.getSpeed()));
      if (shootSpeedContller.atSetpoint()){
          if (oi.shoot()){
            RobotContainer.INTAKE.spinBelt(beltSpeed);
            startTime =  System.currentTimeMillis();
            shootTime = startTime + timeNeeded;
            if (currentTime > shootTime){
              RobotContainer.INTAKE.spinBelt(0);
              RobotContainer.INTAKE.spinIntake(0);
              hasShot = true;
            }

          }
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSpeedController.reset();
    shootSpeedContller.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotSpeedController.atSetpoint() && shootSpeedContller.atSetpoint() && hasShot && hasReved;
  }
}
