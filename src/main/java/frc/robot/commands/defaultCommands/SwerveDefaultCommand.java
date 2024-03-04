// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class SwerveDefaultCommand extends Command {

  private final SwerveChassisSubsystem swerveSubsystem;
  private final OI oi;
  

  /** Creates a new SwerveDefaultCommand. */
  public SwerveDefaultCommand(SwerveChassisSubsystem swerveSubsystem,
  OI opInput) {
    this.swerveSubsystem = swerveSubsystem;
    
    this.oi = opInput;
    swerveSubsystem.commandActive = true;

    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.straightenModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Field Oriented Mode?", swerveSubsystem.getIsFieldOriented());


    if (oi.toggleFieldOriented()) {
      swerveSubsystem.toggleFieldOriented();
    }

    if (oi.resetGyro()) {
      swerveSubsystem.zeroHeading();
    }

    double xSpeed = oi.xSpeed(); // *0.5
    double ySpeed = oi.ySpeed(); // *0.5
    double turnSpeed = oi.rotate(); // *0.1

    if (xSpeed < 0.05 && xSpeed > -0.05) {
      xSpeed = 0;
    }

    if (ySpeed < 0.05 && ySpeed > -0.05) {
      ySpeed = 0;
    }

    if (turnSpeed < 0.05 && turnSpeed > -0.05) {
      turnSpeed = 0;
    }

    
    //double speedIncrease = speedBoost.get();
    //double speedDecrease = speedDampener.get();
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", turnSpeed);
        


    swerveSubsystem.driveSwerve(xSpeed, ySpeed, turnSpeed);
  
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


