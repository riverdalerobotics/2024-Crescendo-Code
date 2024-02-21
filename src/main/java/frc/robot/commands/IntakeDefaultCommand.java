// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeDefaultCommand extends Command {
  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.INTAKE);
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: this is for testing only
    RobotContainer.INTAKE.spinIntake(RobotContainer.OI.shoot());
    RobotContainer.INTAKE.spinBelt(RobotContainer.OI.beltSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
