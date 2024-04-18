// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AutoShootFromPredefinedDistance extends Command {
  /** Creates a new AutoShootFromPredefinedDistance. */
  PivotSubsystem pivot;
  PIDController pivotArmController;
  double distance;
  double setpoint;
  /**
   * Pivots the arm to a angle when given the distance
   * @param pivotSubsystem
   * @param distance in meters
   */
  public AutoShootFromPredefinedDistance(PivotSubsystem pivotSubsystem, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivotSubsystem;
    addRequirements(pivot);
    pivotArmController = new PIDController(PivotConstants.PIDConstants.kPivotP, PivotConstants.PIDConstants.kPivotI, PivotConstants.PIDConstants.kPivotD);
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distanceFromMeters = HelperMethods.unitConvertionMetersToFt(distance);
    setpoint = (9.5/5)*distanceFromMeters;
    pivot.setPivotTolerance(Units.degreesToRotations(PivotConstants.PIDConstants.kHighFeedTolerance));
    pivot.specCommandRunning = true;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      pivot.setPivotAngleDegrees(setpoint);
      SmartDashboard.putNumber("Pivot/PivotSetpoint", setpoint); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotArmController.reset();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotArmController.atSetpoint();
    
  
  }
}
