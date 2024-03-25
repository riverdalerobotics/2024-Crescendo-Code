// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;
import frc.robot.subsystems.SwerveModule;

public class DriveXMetersForward extends Command {
  double startXPos;
  double desiredXPos;
  SwerveModule frontLeftModule;
  SwerveChassisSubsystem swerve;
  PIDController xController;
  double driveMeters;
  /** Creates a new DriveXMetersForward. */
  public DriveXMetersForward(SwerveChassisSubsystem swerve, double driveMeters) {
    this.swerve = swerve;
    xController = new PIDController(CommandConstants.kXNoteAlignP, CommandConstants.kXNoteAlignI, CommandConstants.kXNoteAlignD);
    addRequirements(swerve);
    frontLeftModule = swerve.getFrontLeftModule();
    xController.setTolerance(CommandConstants.kXNoteAlignTolerance);
    this.driveMeters = driveMeters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.enableRobotOriented();
    startXPos = frontLeftModule.getDrivePosition();
    desiredXPos = startXPos + driveMeters;
    xController.setSetpoint(desiredXPos);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.driveSwerve(xController.calculate(frontLeftModule.getDrivePosition()), 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xController.reset();
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint();
  }
}
