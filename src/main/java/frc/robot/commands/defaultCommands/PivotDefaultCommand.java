// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//feed forward is hold val
//cosine ratio feed forward for arm
//motion talonfx gradual PID setpoint change

package frc.robot.commands.defaultCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
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
  double hardStopPosition = PivotConstants.kMinPivotRotationDegrees;

  //TODO: find good tolerance value
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;

  //This tracks the current desired angle the arm is heading towards in degrees
  double desiredArmAngle;

  //The operator can only manually control the pivot when this is true
  boolean manualRotationEnabled = false;

  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
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
    //Sets the desired angle to the current arm angle when this command is initialized
    //If a command that sets the arm position ends, the default command will continue holding that position
    

    desiredArmAngle = pivot.getEncoders();
    angleController.setSetpoint(desiredArmAngle);
    angleController.setTolerance(tolerance);
    manualRotationEnabled = false;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: get someone to read this over please cause it might not be worth doing...
    //System.out.println(angleController.getPositionError());
    System.out.println(pivot.getEncoders());
    System.out.println(angleController.getSetpoint());

    //Manual rotation will stop whatever desired angle the arm is currently heading towards
    if(operatorInput.enableManualRotation()) {
      manualRotationEnabled = true;
      desiredArmAngle = pivot.getEncoders();
    }


    if(manualRotationEnabled) {

      //TODO: increase this value after testing
      desiredArmAngle = HelperMethods.limitValInRange(PivotConstants.PIDConstants.kMinSetpoint, PivotConstants.PIDConstants.kMaxSetpoint, desiredArmAngle + (operatorInput.pivotArm() * 0.25));

      
    }


    if(operatorInput.pivotToIntakePosition()) {
      desiredArmAngle = PivotConstants.kIntakeAngle;
      manualRotationEnabled = false;
    }


    else if(operatorInput.shootPos()) {

      desiredArmAngle = PivotConstants.kSubwooferShootAngle;
      manualRotationEnabled = false;
    }

    else if (operatorInput.pivotToFeed()) {
      desiredArmAngle = PivotConstants.kFeedAngle;
      manualRotationEnabled = false;
    }

    /**
    if (operatorInput.resetArmMinPos()) {
      pivot.setPivotEncoder(PivotConstants.PIDConstants.kMinSetpoint);
    }
    else if (operatorInput.resetArmMaxPos()) {
      pivot.setPivotEncoder(PivotConstants.PIDConstants.kMaxSetpoint);
    }*/

    
    angleController.setSetpoint(desiredArmAngle);

    //TODO: Commented out for testing
    //pivot.movePivot(angleController.calculate(pivot.getEncoders()));
    pivot.movePivot(HelperMethods.limitValInRange(PivotConstants.PIDConstants.kPivotPIDMinOutput, PivotConstants.PIDConstants.kPivotPIDMaxOutput, angleController.calculate(pivot.getEncoders())));


    //Voltage above max voltage indicates that the arm is pushing against the hard stop and should be reset
    //TODO: Test to see if this could be screwed up by other robots or field elements. If it can, we need to ensure this doesn't 
    //unintenionally result in robot death
    
    if (pivot.getCurrent() > maxCurrent){
      angleController.setSetpoint(hardStopPosition);
      desiredArmAngle = hardStopPosition;
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
