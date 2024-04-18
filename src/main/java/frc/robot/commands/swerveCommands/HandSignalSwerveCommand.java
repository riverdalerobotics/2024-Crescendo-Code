// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;
import frc.robot.subsystems.SwerveModule;

public class HandSignalSwerveCommand extends Command {
  double startXPos;
  double desiredXPos;
  SwerveModule frontLeftModule;
  SwerveChassisSubsystem swerve;
  Limelight limelight;
  
  /** Creates a new DriveXMetersForward. */
  public HandSignalSwerveCommand(SwerveChassisSubsystem swerve, Limelight limelight) {
    this.swerve = swerve;
    this.limelight = limelight;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.enableRobotOriented();
    swerve.setDrivesBrake();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (limelight.getClassName()=="forwards"){
    swerve.driveSwerve(0.1,0,0);
  } else if (limelight.getClassName()=="backwards"){
    swerve.driveSwerve(-0.1, 0, 0);
  } else if (limelight.getClassName()=="left"){
    swerve.driveSwerve(0, 0.1, 0);
  }  else if (limelight.getClassName()=="right"){
    swerve.driveSwerve(0, -0.1, 0);
  } else if (limelight.getClassName()=="nothing"){
    swerve.driveSwerve(0,0,0);
  }
 
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setModuleXPosition();
    swerve.setDrivesCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
