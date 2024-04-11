// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveCommands;

import org.ejml.sparse.csc.decomposition.lu.LuUpLooking_FSCC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HelperMethods;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.SwerveChassisSubsystem;

public class AutoFaceSpeakerCommand extends Command {
  /** Creates a new AutoFaceSpeakerCommand. */
  SwerveChassisSubsystem chassis;
  PIDController turnController;
  Limelight tagLimelight;
  OI oi;
  public AutoFaceSpeakerCommand(SwerveChassisSubsystem chassisSubsystem, Limelight tagLimelight, OI oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassisSubsystem;
    addRequirements(chassis);
    this.tagLimelight = tagLimelight;
    this.oi = oi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
    turnController.setSetpoint(CommandConstants.kTurningTagAlignSetpoint);
    turnController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean tagDetected = tagLimelight.targetDetected();
    if(tagDetected){
      double tagThetaOffset = tagLimelight.getTX();
      double turnPIDspeed = turnController.calculate(tagThetaOffset);
      double turnSpeed = HelperMethods.limitValInRange(CommandConstants.kTurningNoteMinOutput, CommandConstants.kTurningNoteMaxOutput, turnPIDspeed);
      chassis.driveSwerve(0,0,turnSpeed);
    }
    else{
      double xSpd = oi.xSpeed();
      double ySpd = oi.ySpeed();
      double turningSpd = oi.rotate();
      chassis.driveSwerve(xSpd, ySpd, turningSpd);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
