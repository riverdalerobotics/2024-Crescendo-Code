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

public class AutoAlignWithNoteSwerve extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController xController;
  private boolean noteIsDetected;
  private double noteYOffset;
  private double noteXOffset;
  private final SwerveChassisSubsystem swerveSubsystem;
  private final Limelight noteLimelight;
  private final OI oi;
  private final BlinkinLED LED;



  public AutoAlignWithNoteSwerve(
    SwerveChassisSubsystem swerveSubsystem,
    OI oi,
    Limelight noteLimelight,
    BlinkinLED LED) {
  
    yController = new PIDController(CommandConstants.kYNoteAlignP, CommandConstants.kYNoteAlignI, CommandConstants.kYNoteAlignD);
    yController.setSetpoint(CommandConstants.kYNoteAlignSetpoint);
    yController.setTolerance(CommandConstants.kYNoteAlignTolerance);

    xController = new PIDController(CommandConstants.kXNoteAlignP, CommandConstants.kXNoteAlignI, CommandConstants.kXNoteAlignD);
    xController.setSetpoint(CommandConstants.kXNoteAlignSetpoint);
    xController.setTolerance(CommandConstants.kXNoteAlignTolerance);


    this.swerveSubsystem = swerveSubsystem;
    this.LED = LED;
    this.noteLimelight = noteLimelight;
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


    noteIsDetected = noteLimelight.targetDetected();
    System.out.println(noteIsDetected);
    SmartDashboard.putBoolean("Note detected", noteIsDetected);

    swerveSubsystem.slowDrive(HelperMethods.applyInputDeadband(oi.engageSlowMode()));

    if (oi.engageDriveBrakeMode()) {
      swerveSubsystem.setDrivesBrake();
    }
    else {
      swerveSubsystem.setDrivesCoast();
    }



    //If a note is detected, turning will be disabled and y movement will be controlled by PID
    if (noteIsDetected) {
      noteYOffset = noteLimelight.getXDisplacementFromNote();
      noteXOffset = -noteLimelight.getYDisplacementFromNote();

      SmartDashboard.putNumber("L/R note displacement", noteYOffset);
      SmartDashboard.putNumber("F/B note displacement", noteXOffset);
      System.out.println(noteXOffset);

      //Field oriented mucks up this command so we disable it when a note is detected
      swerveSubsystem.enableRobotOriented();

      //Calculate pid input using lr offset from note and limit it to max speed values to avoid overshooting
      double PIDySpeed = yController.calculate(noteYOffset);
      PIDySpeed = HelperMethods.limitValInRange(CommandConstants.kYNoteAlignMinOutput, CommandConstants.kYNoteAlignMaxOutput, PIDySpeed);
      double PIDxSpeed = xController.calculate(noteXOffset);
      PIDxSpeed = HelperMethods.limitValInRange(CommandConstants.kXNoteAlignMinOutput, CommandConstants.kXNoteAlignMaxOutput, PIDxSpeed);
     
      swerveSubsystem.driveSwerveWithPhysicalMax(PIDxSpeed, PIDySpeed, 0);
      
//      SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    }
    
    
    //If no note is detected, drive like normal
    else {
      swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
    }
    SmartDashboard.putBoolean("Is at note", xController.atSetpoint());
    //System.out.println(xController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (xController.atSetpoint() && yController.atSetpoint()) {
      LED.enableAutoAlignCompleteLED();
    }
    yController.reset();
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
