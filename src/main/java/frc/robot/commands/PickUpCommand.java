// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
public class PickUpCommand extends Command {
  PIDController pivotController;
  double kp = PivotConstants.PIDConstants.kPivotP;
  double ki = PivotConstants.PIDConstants.kPivotI;
  double kd = PivotConstants.PIDConstants.kPivotD;
  double setpoint = 0d;
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double intakeSpeed = IntakeConstants.kDesiredIntakeMotorRPS;
  double beltSpeed = IntakeConstants.kIntakeBeltMotorSpeed;
  double maxVoltage = 0d;
  boolean hasPickedUp = false;
  

  /** Creates a new AutoPickUpCommand. */
  public PickUpCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.PIVOT);
    addRequirements(RobotContainer.INTAKE);
    pivotController = new PIDController(kp, ki, kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotController.setSetpoint(setpoint);
    pivotController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.PIVOT.movePivot(pivotController.calculate(RobotContainer.PIVOT.getEncoders()));
    if(pivotController.atSetpoint()){
      RobotContainer.INTAKE.spinIntake(intakeSpeed);
      RobotContainer.INTAKE.spinBelt(beltSpeed);
      if (RobotContainer.INTAKE.intakeVoltage() < maxVoltage){
        RobotContainer.INTAKE.spinIntake(0);
        hasPickedUp = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasPickedUp == true && pivotController.atSetpoint();
  }
}
