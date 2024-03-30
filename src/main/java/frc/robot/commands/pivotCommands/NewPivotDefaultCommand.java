// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//feed forward is hold val
//cosine ratio feed forward for arm
//motion talonfx gradual PID setpoint change

package frc.robot.commands.pivotCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.OI;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class NewPivotDefaultCommand extends Command {
  /** Creates a new PivotDefaultCommand. */
  PivotSubsystem pivot;
  OI operatorInput;
  double hardStopPosition = PivotConstants.PIDConstants.kMinSetpoint;

  //TODO: find good tolerance value
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double requestedArmAngle = PivotConstants.kZeroAngle;


  //The operator can only manually control the pivot when this is true
  boolean manualRotationEnabled = false;

  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  public NewPivotDefaultCommand(OI opInput, PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.pivot = pivot;
    this.operatorInput = opInput;
    addRequirements(pivot);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets the desired angle to the current arm angle when this command is initialized
    //If a command that sets the arm position ends, the default command will continue holding that position
    
    //When this command begins, it stops the movement of the arm by setting the desired position
    //to it's current position
    pivot.setPivotAngleDegrees(pivot.getEncoders());
    manualRotationEnabled = false;
    requestedArmAngle = pivot.getEncoders();
    System.out.println("def command pivot start");
    pivot.specCommandRunning = false;

    
  }

  //TODO: on enable, set arm to current encoder position to avoid it trying to return to an old value
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //Manual rotation will stop whatever desired angle the arm is currently heading towards
    if(operatorInput.enableManualRotation()) {
      manualRotationEnabled = true;
    }


    if(manualRotationEnabled) {
      
      requestedArmAngle = requestedArmAngle + (HelperMethods.applyInputDeadband(operatorInput.pivotArm()) * 0.25);
      pivot.setPivotAngleDegreesNoLimit(requestedArmAngle);
    }
    else {
      requestedArmAngle = HelperMethods.limitValInRange(PivotConstants.PIDConstants.kMinSetpoint, PivotConstants.PIDConstants.kMaxSetpoint, requestedArmAngle);
      pivot.setPivotAngleDegrees(requestedArmAngle);
    }




    if (operatorInput.resetArmMinPos()) {
      pivot.resetPivotEncoder();
      requestedArmAngle = PivotConstants.kZeroAngle;
      manualRotationEnabled = false;
    }

    if (operatorInput.resetArmMaxPos()) {
      pivot.setPivotEncoder(PivotConstants.PIDConstants.kMaxSetpoint);
      requestedArmAngle = PivotConstants.PIDConstants.kMaxSetpoint;
      manualRotationEnabled = false;
    }





    //Voltage above max voltage indicates that the arm is pushing against the hard stop and should be reset
    //TODO: Test to see if this could be screwed up by other robots or field elements. If it can, we need to ensure this doesn't 
    //unintenionally result in robot death
    //TODO: Fix the logic here. Probably have it activate the tuck command to reset or figure out which hard stop is being pressed and change encoder pos to it
    
    /*if (pivot.getCurrent() > maxCurrent){
      requestedArmAngle = hardStopPosition;
      pivot.setPivotAngleDegrees(requestedArmAngle);
    }*/
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
