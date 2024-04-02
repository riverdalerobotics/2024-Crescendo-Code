// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;



//NOTE: THIS COMMAND IS CURRENTLY UNUSED IN THE ROBOT BUILD AS OF APRIL 2nd

/**
 * This command brings the intake down and spins the flywheels until current spikes, indicating a
 * note has been picked up. 
 */
public class PickUpCommand extends Command {
  PIDController pivotController;
  double kp = PivotConstants.PIDConstants.kPivotP;
  double ki = PivotConstants.PIDConstants.kPivotI;
  double kd = PivotConstants.PIDConstants.kPivotD;
  double setpoint = PivotConstants.kIntakeAngle;
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double intakeSpeed = IntakeConstants.kDesiredIntakeMotorRPS;
  double beltSpeed = IntakeConstants.kIntakeBeltMotorSpeed;
  double maxCurrent = IntakeConstants.kIntakeCurrentThreshold;
  boolean hasPickedUp = false;
  IntakeSubsystem intake;
  PivotSubsystem pivot;
  
  
  /** Creates a new AutoPickUpCommand. */
  public PickUpCommand(IntakeSubsystem intake, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.intake = intake;
    
    addRequirements(pivot, intake);
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

    //Intake only begins spinning after the pivot has made it to position
    pivot.movePivot(pivotController.calculate(pivot.getEncoders()));
    if(pivotController.atSetpoint()){
      intake.spinIntake(intakeSpeed);
      intake.spinBelt(beltSpeed);


      //TODO: Create a new thread that sleeps then disables the intake spin
      //The intake current will spike when the note enters the intake.
      if (intake.flywheelTorqueCurrent() > maxCurrent){
        intake.spinIntake(0);
        intake.spinBelt(0);
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
