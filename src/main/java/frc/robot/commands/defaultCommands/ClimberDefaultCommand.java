// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */
  OI operatorInput;
  public ClimberDefaultCommand(OI opInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.CLIMB);
    this.operatorInput = opInput;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
