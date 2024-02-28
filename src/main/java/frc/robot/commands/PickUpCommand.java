// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import java.util.concurrent.TimeUnit;
public class PickUpCommand extends Command {
  PIDController speedController;
  double kp = 0d;
  double ki = 0d;
  double kd = 0d;
  double setpoint = 0d;
  double tolerance = 0d;
  double intakeSpeed = 0d;
  double beltSpeed = 0d;
  double maxVoltage = 0d;
  boolean hasPickedUp = false;
  

  /** Creates a new AutoPickUpCommand. */
  public PickUpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(RobotContainer.PIVOT);
    //addRequirements(RobotContainer.INTAKE);
    speedController = new PIDController(kp, ki, kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speedController.setSetpoint(setpoint);
    speedController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*RobotContainer.PIVOT.movePivot(speedController.calculate(RobotContainer.PIVOT.getEncoders()));
    if(speedController.atSetpoint()){
      RobotContainer.INTAKE.spinIntake(intakeSpeed);
      RobotContainer.INTAKE.spinBelt(beltSpeed);
      if (RobotContainer.INTAKE.intakeVoltage() < maxVoltage){
        RobotContainer.INTAKE.spinIntake(0);
        RobotContainer.INTAKE.spinBelt(0);
        hasPickedUp = true;
      }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    speedController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPickedUp == true && speedController.atSetpoint();
  }
}
