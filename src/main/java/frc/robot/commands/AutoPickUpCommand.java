// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.Limelight;
import frc.robot.Constants.CommandConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

//have pid for all swerve controls
//Once all are close to desired val, turn on intake and drive forward


public class AutoPickUpCommand extends Command {
  /** Creates a new AutoAlignWithNoteSwerve. */
  private PIDController yController;
  private PIDController turningController;
  private PIDController xController;


  //Limelight values conveying if a note is detected & if it is, the distance values from it to the robot
  private boolean noteIsDetected;
  private double noteYOffset;
  private double noteThetaOffset;
  private double noteXOffset;

  //When all PIDControllers have reached their setpoints, begin the pickup sequence by moving forward slightly and powering intake
  private boolean beginPickupSequence;

  private final SwerveChassisSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final Limelight noteLimelight;

  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;

  private final Supplier<Boolean> toggleSlowModeFunction;

  



  public AutoPickUpCommand(
    SwerveChassisSubsystem swerveSubsystem, 
    IntakeSubsystem intakeSubsystem,
    Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction, 
    Supplier<Boolean> toggleSlow,
    Limelight noteLimelight) {

    yController = new PIDController(CommandConstants.kYNoteAlignP, CommandConstants.kYNoteAlignI, CommandConstants.kYNoteAlignD);
    yController.setSetpoint(CommandConstants.kYNoteAlignSetpoint);
    yController.setTolerance(CommandConstants.kYNoteAlignTolerance);

    turningController = new PIDController(CommandConstants.kTurningNoteAlignP, CommandConstants.kTurningNoteAlignI, CommandConstants.kTurningNoteAlignD);
    turningController.setSetpoint(CommandConstants.kTurningNoteAlignSetpoint);
    turningController.setTolerance(CommandConstants.kTurningNoteAlignTolerance);

    xController = new PIDController(CommandConstants.kXNoteAlignP, CommandConstants.kXNoteAlignI, CommandConstants.kXNoteAlignD);
    xController.setSetpoint(CommandConstants.kXNoteAlignSetpoint);
    xController.setTolerance(CommandConstants.kXNoteAlignTolerance);


    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.toggleSlowModeFunction = toggleSlow;

    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.noteLimelight = noteLimelight;
    addRequirements(swerveSubsystem, intakeSubsystem);
  /** Creates a new AutoPickUpCommand. */
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    beginPickupSequence = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turningSpd = turningSpdFunction.get();
    noteIsDetected = noteLimelight.targetDetected();


    if (beginPickupSequence) {
      intakeSubsystem.spinIntake(1);
      //should probably do an odometry move forward here but it can be timed until that exists
      //Could also do some PID jank
    }
    else {

      if (fieldOrientedFunction.get() && noteIsDetected == false) {
        swerveSubsystem.toggleFieldOriented();
      }
      
      if (toggleSlowModeFunction.get()) {
        //Toggle slow
      }

      //If a note is detected, turning will be disabled and y movement will be controlled by PID
      if (noteIsDetected) {

        //y and x being swapped is intentional. Robot x and y is opposite of limelight x and y
        noteYOffset = noteLimelight.getXDisplacementFromNote();
        noteThetaOffset = noteLimelight.getTX();
        noteXOffset = noteLimelight.getYDisplacementFromNote();
        //Field oriented mucks up this command so we disable it when a note is detected
        swerveSubsystem.disableFieldOriented();
        double PIDYSpeed = yController.calculate(noteYOffset);
        double PIDTurningSpeed = turningController.calculate(noteThetaOffset);
        double PIDXSpeed = xController.calculate(noteXOffset);
        swerveSubsystem.driveSwerve(PIDXSpeed, PIDYSpeed, PIDTurningSpeed);


        if (yController.atSetpoint() && xController.atSetpoint() && turningController.atSetpoint()) {
          beginPickupSequence = true;
        }
      }

      //If no note is detected, drive like normal
      else {
        swerveSubsystem.driveSwerve(xSpd, ySpd, turningSpd);
        
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // isFinished should be able to return true right?
  @Override
  public boolean isFinished() {
    return false;
  }
}
