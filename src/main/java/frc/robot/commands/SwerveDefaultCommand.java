// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class SwerveDefaultCommand extends Command {

  private final SwerveChassisSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final Supplier<Boolean> toggleSlowModeFunction;
  private boolean isFieldOriented;


  /** Creates a new SwerveDefaultCommand. */
  public SwerveDefaultCommand(SwerveChassisSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> toggleSlow) {
    this.swerveSubsystem = swerveSubsystem;
    
    this.isFieldOriented = true;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.toggleSlowModeFunction = toggleSlow;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    swerveSubsystem.commandActive = true;

    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Field Oriented Mode?", isFieldOriented);

    if (toggleSlowModeFunction.get()) {
      //Toggle slow
    }

    if (fieldOrientedFunction.get()) {
      swerveSubsystem.toggleFieldOriented();


    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turnSpeed = turningSpdFunction.get();

    
    //double speedIncrease = speedBoost.get();
    //double speedDecrease = speedDampener.get();
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", turnSpeed);
        


    swerveSubsystem.driveSwerve(xSpeed, ySpeed, turnSpeed);
  }
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


