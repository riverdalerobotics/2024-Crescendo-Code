// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class RotateToShuttle extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController turningController;
  private PIDController xController;

  private boolean noteIsDetected;
  private double angleOffset;
  private final SwerveChassisSubsystem swerveSubsystem;
  
  private final OI oi;
  private final BlinkinLED LED;


  //TODO: Consider using slew rate limiters here
  public RotateToShuttle(
    SwerveChassisSubsystem swerveSubsystem,
    OI oi,
    BlinkinLED LED) {
    
    turningController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
    turningController.setSetpoint(60);
    turningController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);

    this.swerveSubsystem = swerveSubsystem;
    this.LED = LED;
    this.oi = oi;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.enableRobotOriented();
  }

  @Override
  public void execute() {
    //System.out.println(xController.atSetpoint());
    double xSpd = oi.xSpeed();
    double ySpd = oi.ySpeed();
    double turningSpd = oi.rotate();

    xSpd = HelperMethods.applyInputDeadband(xSpd);
    ySpd = HelperMethods.applyInputDeadband(ySpd);
    turningSpd = HelperMethods.applyInputDeadband(turningSpd);


    
    SmartDashboard.putBoolean("Note detected", noteIsDetected);

    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));



    //if b button is presed, rotate robot to shuttle angle, 
    if (oi.rotateRobot() == true) {
    
      double offset = swerveSubsystem.getHeadingDegrees();

      double PIDThetaSpeed = turningController.calculate(offset);
      PIDThetaSpeed = HelperMethods.limitValInRange(CommandConstants.kTurningNoteMinOutput, CommandConstants.kTurningNoteMaxOutput, PIDThetaSpeed);
      
      swerveSubsystem.driveSwerve(xSpd, ySpd, PIDThetaSpeed*0.1);

 

      
//      SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    }
    
    
    //If no note is detected, drive like normal
    else {
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
    SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    System.out.println(xController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (xController.atSetpoint() && yController.atSetpoint()) {
      LED.enableAutoAlignCompleteLED();
    }
    yController.reset();
    turningController.reset();
    xController.reset();
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint()) {
      return true;
    }
    else {
      return false;
    }
  }
}
