// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotToAngle extends Command {
  /** Creates a new AutoPivotToAngle. */
  double tolerance = PivotConstants.PIDConstants.kPivotToleranceThreshold;
  double desiredAngle;
  PivotSubsystem pivot;
  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  double hardStopPosition = PivotConstants.PIDConstants.kMinSetpoint;
  public AutoPivotToAngle(double angle, PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivotSubsystem;
    this.desiredAngle = angle;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPivotAngleDegrees(desiredAngle);
    
    //The constant tolerance value is in degrees, but the internal controller uses rotations.
    //We convert the degrees tolerance into rotations so the internal motor controller can work with the value
    pivot.setPivotTolerance(Units.degreesToRotations(tolerance));
    pivot.specCommandRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override  
  public void execute() {
  
    //TODO: Fix the logic for this because it kind of sucks rn (if encoder is off the correct value this will not work)
    if (pivot.getCurrent() > maxCurrent){
      pivot.setPivotAngleDegrees(hardStopPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.getPivot1().atSetpointPosition(Units.degreesToRotations(desiredAngle));
  }
}