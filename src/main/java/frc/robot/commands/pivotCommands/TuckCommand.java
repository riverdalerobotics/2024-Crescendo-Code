// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

/**
 * This command resets the arm pivot from anywhere, returning it to the starting hard stop position
 */
public class TuckCommand extends Command {
  /** Creates a new AutoTuckCommand.  */
  PIDController speedController;
  double setpoint = 0d;
  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  double speed = 0d;
  boolean hasStopped = false;
  PivotSubsystem pivot;
  double waitTime = 0;
  boolean commandComplete;
  public TuckCommand(PivotSubsystem pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    // double kp = 0d;
    // double ki = 0d;
    // double kd = 0d;
    this.pivot = pivot;
    addRequirements(pivot);
    // speedController = new PIDController(kp, ki, kd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //   speedController.setSetpoint(setpoint);
  //   speedController.setTolerance(tolerance);
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  //TODO: Remove PID controller and just supply a constant speed to the pivot until it hits the hard stop
  public void execute() {

    //When the arm pushes against the hard stop, stop the arm.
    if (hasStopped == false) {
      pivot.movePivot(speed);
      if(pivot.getCurrent() > maxCurrent){
        pivot.movePivot(0);
        hasStopped = true;
      }
    }

    //wait half a second before resetting the encoder to ensure the rio has received the updated
    //position
    if (hasStopped && waitTime == 0) {
      waitTime = System.currentTimeMillis() + 500;
    }

    if (hasStopped && System.currentTimeMillis() > waitTime) {
      pivot.resetPivotEncoder();
      commandComplete = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //speedController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandComplete;
  }
}
