// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.OI;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class SwerveDefaultCommand extends Command {

  private final SwerveChassisSubsystem swerveSubsystem;
  private final OI oi;
  

  /** Creates a new SwerveDefaultCommand. */
  public SwerveDefaultCommand(
    SwerveChassisSubsystem swerveSubsystem,
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


    //The left joystick engages field oriented when pressed. The right joystick disables it
    //Field oriented means that when the left joystick is pressed up, the robot will move forwards
    //relative to what the drivers see as forwards
    //robot oriented would move forward depending on the current orientation of the robot
    //We avoid using a single toggle button as it creates confusion for drivers
    if (oi.engageFieldOriented()) {
      swerveSubsystem.enableFieldOriented();
    }
    else if (oi.engageRobotOriented()) {
      swerveSubsystem.enableRobotOriented();
    }
    

    //This can be used to change what forward is for field oriented
    if (oi.resetGyro()) {
      swerveSubsystem.zeroHeading();
    }

    if (oi.engageDriveBrakeMode()) {
      swerveSubsystem.setDrivesBrake();
    }
    else {
      swerveSubsystem.setDrivesCoast();
    }

    //goes to brake mode when slow mode is on (left bumper pressed)
    if (oi.engageSlowMode() == 1 ) {
      swerveSubsystem.setDrivesBrake();
    }
    else {
      swerveSubsystem.setDrivesCoast();
    }


    //hold left bumper to slow the robot down
    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));

    double xSpeed = oi.xSpeed(); 
    double ySpeed = oi.ySpeed(); 
    double turnSpeed = oi.rotate(); 

    //Some joysticks will have small values even when they're not being pressed
    //This method applies an input deadband to account for this, setting small input values to 0
    xSpeed = HelperMethods.applyInputDeadband(xSpeed);
    ySpeed = HelperMethods.applyInputDeadband(ySpeed);
    turnSpeed = HelperMethods.applyInputDeadband(turnSpeed);


    

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", turnSpeed);
        


    swerveSubsystem.driveSwerve(xSpeed, ySpeed, turnSpeed);
    
    
    if (oi.engageXModulePosition()) {
      swerveSubsystem.setModuleXPosition();
    }
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //Drive speed should always be reset in any method that has the option to use slow mode
    swerveSubsystem.stopModules();
    swerveSubsystem.resetDriveSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


