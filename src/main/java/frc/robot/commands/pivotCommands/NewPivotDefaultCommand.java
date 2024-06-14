// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//feed forward is hold val
//cosine ratio feed forward for arm
//motion talonfx gradual PID setpoint change

package frc.robot.commands.pivotCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  boolean countingHardStop;
  double hardStopTimer;


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

    hardStopTimer = 0;
    countingHardStop = false;
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
      
      requestedArmAngle = requestedArmAngle + (HelperMethods.applyInputDeadband(operatorInput.pivotArm()) * 0.15);
      pivot.setPivotAngleDegreesNoLimit(requestedArmAngle);
    }
    else {
      requestedArmAngle = HelperMethods.limitValInRange(PivotConstants.PIDConstants.kMinSetpoint, PivotConstants.PIDConstants.kMaxSetpoint, requestedArmAngle);
      pivot.setPivotAngleDegrees(requestedArmAngle);
    }




    // if (operatorInput.resetArmMinPos()) {
    //   pivot.resetPivotEncoder();
    //   requestedArmAngle = PivotConstants.kZeroAngle;
    //   manualRotationEnabled = false;
    // }

    // if (operatorInput.resetArmMaxPos()) {
    //   pivot.setPivotEncoder(PivotConstants.PIDConstants.kMaxSetpoint);
    //   requestedArmAngle = PivotConstants.PIDConstants.kMaxSetpoint;
    //   manualRotationEnabled = false;
    // }


    

    //Checks if the pivot is above the threshold indicating it is pushing into a hardstop
    if (pivot.getCurrent() > PivotConstants.kHardStopCurrentThreshold) {
      //Runs once every time current breaches the limit. Once current goes below the limit, this statement can run again
      if (countingHardStop == false) {
        hardStopTimer = System.currentTimeMillis() + 750;
        countingHardStop = true;

      }
    } 
    //If current returns to accepted values within 1 second of breaching, reset the timer and boolean
    else {
      countingHardStop = false;
      hardStopTimer = 0;
    }



    //Checks if the current spike has been breached for a second
    //if so, ends the command and resets the arm
    if (countingHardStop && System.currentTimeMillis() > hardStopTimer) {
      System.out.println("WE PUSHING P WITH THIS ONE AND BY P I MEAN HARDSTOP");

      //Checks which hard stop is being pushed into by checking the direction of the motors
      if (pivot.getPivot1().getDutyCycle().getValueAsDouble() > 0) {
        //pivot.setPivotEncoder(PivotConstants.PIDConstants.kMaxSetpoint);
        pivot.setPivotEncoder(29);
        requestedArmAngle = PivotConstants.PIDConstants.kMaxSetpoint;
        manualRotationEnabled = false;
        System.out.println("intake reset pivot");
      }
      else if (pivot.getPivot1().getDutyCycle().getValueAsDouble() < 0) {
        pivot.setPivotEncoder(-152);
        requestedArmAngle = PivotConstants.PIDConstants.kMinSetpoint;
        manualRotationEnabled = false;
        System.out.println("stow reset pivot");
      
      }

    
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
