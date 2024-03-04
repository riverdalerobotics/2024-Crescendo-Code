// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotDefaultCommand extends Command {
  /** Creates a new PivotDefaultCommand. */
  PivotSubsystem pivot;
  OI operatorInput;
  PIDController angleController;
  double kp = PivotConstants.PIDConstants.kPivotP;
  double ki = PivotConstants.PIDConstants.kPivotI;
  double kd = PivotConstants.PIDConstants.kPivotD;
  double setpoint = PivotConstants.kMinPivotRotationDegrees;

  //TODO: find good tolerance value
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;

  //This tracks the current desired angle the arm is heading towards in degrees
  double desiredArmAngle = setpoint;

  //The operator can only manually control the pivot when this is true
  boolean manualRotationEnabled = false;

  double maxVoltage = PivotConstants.kPivotMaxVoltage;
  public PivotDefaultCommand(OI opInput, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.pivot = pivot;
    this.operatorInput = opInput;
    angleController = new PIDController(kp, ki, kd);
    addRequirements(pivot);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(setpoint);
    angleController.setTolerance(tolerance);
    desiredArmAngle = setpoint;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: get someone to read this over please cause it might not be worth doing...


    //Manual rotation will stop whatever desired angle the arm is currently heading towards
    if(operatorInput.enableManualRotation()) {
      manualRotationEnabled = true;
      desiredArmAngle = pivot.getEncoders();
    }


    if(manualRotationEnabled) {

      //TODO: increase this value after testing
      desiredArmAngle += (operatorInput.pivotArm() * 0.1);
      
    }


    if(operatorInput.pivotToIntakePosition()) {
      desiredArmAngle = PivotConstants.intakeAngle;
      manualRotationEnabled = false;
    }



    angleController.setSetpoint(desiredArmAngle);
    pivot.movePivot(angleController.calculate(pivot.getEncoders()));


    //Voltage above max voltage indicates that the arm is pushing against the hard stop and should be reset
    //TODO: Test to see if this could be screwed up by other robots or field elements. If it can, we need to ensure this doesn't 
    //unintenionally result in robot death
    if (pivot.getVoltage() < maxVoltage){
      angleController.setSetpoint(setpoint);
      desiredArmAngle = setpoint;
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
