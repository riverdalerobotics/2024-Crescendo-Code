// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class TuckCommand extends Command {
  /** Creates a new AutoTuckCommand.  */
  PIDController speedController;
  double setpoint = 0d;
  double tolerance = 0d;
  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  double speed = 0d;
  boolean hasStoped = false;
  PivotSubsystem pivot;
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


    pivot.movePivot(speed);
    if( pivot.getCurrent() > maxCurrent){
      pivot.resetPivotEncoder();
      pivot.movePivot(0);
      hasStoped = true;
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
    return hasStoped;
  }
}
