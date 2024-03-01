// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import javax.xml.crypto.dsig.spec.C14NMethodParameterSpec;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PivotSubsytem;

public class PivotDefaultCommand extends Command {
  /** Creates a new PivotDefaultCommand. */
  PivotSubsytem pivot;
  OI operatorInput;
  PIDController angleController;
  double kp = 0d;
  double ki = 0d;
  double kd = 0d;
  double setpoint = 0d;
  double tolerance = 0d;
  double changeArmAngle;
  double maxVoltage = 0d;
  public PivotDefaultCommand(OI opInput, PivotSubsytem pivOT) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.pivot = pivOT;
    this.operatorInput = opInput;
    angleController = new PIDController(kp, ki, kd);
    addRequirements(RobotContainer.PIVOT);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(setpoint);
    angleController.setTolerance(tolerance);
     changeArmAngle = setpoint;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: get someone to read this over please cause it might not be worth doing...
    pivot.movePivot(angleController.calculate(pivot.getEncoders()));
    if(operatorInput.moveArmUp()){
      changeArmAngle += 1;
      angleController.setSetpoint(changeArmAngle);
    }
    else if(operatorInput.moveArmDown()){
      changeArmAngle -= 1;
      angleController.setSetpoint(changeArmAngle);
    }
    if (pivot.getVoltage()<maxVoltage){
      angleController.setSetpoint(setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
