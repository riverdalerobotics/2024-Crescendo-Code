// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.combinationCommands.AutoAlignAndPickUp;
import frc.robot.commands.combinationCommands.IntakeIndefinitelyCommand;
import frc.robot.commands.combinationCommands.PivotToAngleAndRevIndefinitely;
import frc.robot.commands.combinationCommands.PivotToAngleAndShoot;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.NewIntakeDefaultCommand;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.commands.pivotCommands.NewPivotDefaultCommand;
import frc.robot.commands.swerveCommands.AutoAlignWithNoteSwerve;
import frc.robot.commands.swerveCommands.DriveXMetersForward;
import frc.robot.commands.swerveCommands.SwerveDefaultCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  
  //This is where we construct all our subsystem and other important objects 
  BlinkinLED LED = new BlinkinLED();
  OI oi = new OI();
  public final Limelight NOTE_LIMELIGHT = new Limelight("limelight-note");
  public final Limelight TAG_LIMELIGHT = new Limelight("limelight-tags"); 
  Autos autoFactory = new Autos();
  public final SwerveChassisSubsystem CHASSIS = new SwerveChassisSubsystem(LED);
  public final PivotSubsystem PIVOT = new PivotSubsystem();
  public final IntakeSubsystem INTAKE = new IntakeSubsystem();
  public final ClimberSubsystem CLIMB = new ClimberSubsystem();
 
  

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    
    // Register Named Commands so that it can be used in the PathPlanning autos 
    //TODO: In Path Planner UI, remember to add the named commands 
    //Path planner is the software we use to make our autos
    NamedCommands.registerCommand("note error fix", new AutoAlignWithNoteSwerve(CHASSIS, oi, NOTE_LIMELIGHT, LED));
    NamedCommands.registerCommand("IntakeIndefinitely", new IntakeIndefinitelyCommand(PIVOT, INTAKE, LED, oi));
    
    NamedCommands.registerCommand("PivotAndRevForShotFront", new PivotToAngleAndRevIndefinitely(PivotConstants.kSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, PIVOT, INTAKE, LED, oi));
    NamedCommands.registerCommand("PivotAndShootFront", new PivotToAngleAndShoot(PivotConstants.kSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, IntakeConstants.kShootBeltMotorSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kPivotToleranceThreshold));

    NamedCommands.registerCommand("PivotAndRevForShotBack", new PivotToAngleAndRevIndefinitely(PivotConstants.kOppositeSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, PIVOT, INTAKE, LED, oi));
    NamedCommands.registerCommand("PivotAndShootBack", new PivotToAngleAndShoot(PivotConstants.kOppositeSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, IntakeConstants.kShootBeltMotorSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    NamedCommands.registerCommand("SetArmUp", new NewAutoPivotToAngle(-90, PIVOT, PivotConstants.PIDConstants.kSetUpTolerance));
    CHASSIS.setDefaultCommand(new SwerveDefaultCommand (
      CHASSIS,
      oi
    ));
  
    
    INTAKE.setDefaultCommand(new NewIntakeDefaultCommand(
      oi,
      INTAKE,
      LED
    ));
    
    PIVOT.setDefaultCommand(new NewPivotDefaultCommand(
      oi, 
      PIVOT
    ));
  }

  /** 
  Put triggers here that change the active commands
  <p>
  Triggers are conditions that activate commands
  <p>
  Triggers can activate as long as an input is true, or toggle on and off based on the input
  <p>
  For more information on triggers, see: https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html
  */
  private void configureBindings() {

    //as long as the right trigger is held, note align will be active
    new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignAndPickUp(CHASSIS, INTAKE, PIVOT, oi, LED, NOTE_LIMELIGHT));

    //as long as the left trigger is held, auto move to predefined will be active
    //new Trigger(() -> oi.engageAutoMoveToPredefined()).whileTrue(new AutoMoveToPredefined(CHASSIS, oi, TAG_LIMELIGHT));
    

    //This command is unfinished, but the purpose is to rezero the arm if the encoder value is innacurate
    new Trigger(() -> oi.tuckArm1()).whileTrue(new NewAutoPivotToAngle(PivotConstants.kZeroAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    new Trigger(() -> oi.tuckArm2()).whileTrue(new NewAutoPivotToAngle(PivotConstants.kZeroAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));

    
    new Trigger(() -> oi.pivotToIntakePosition()).onTrue(new NewAutoPivotToAngle(PivotConstants.kIntakeAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    new Trigger(() -> oi.engageIntake()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredIntakeMotorRPS, IntakeConstants.kIntakeBeltMotorSpeed, INTAKE, LED, oi));


    //Feed controls do not require driver to press the fire button.
    //They will shoot as soon as intake and angle are prepared
    new Trigger(() -> oi.pivotAndShootLowFeed()).whileTrue(new PivotToAngleAndShoot(PivotConstants.kFeedAngle, IntakeConstants.kDesiredFeedMotorRPS, IntakeConstants.kDesiredFeedBeltSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, false, PivotConstants.PIDConstants.kLowFeedTolerance));
    new Trigger(() -> oi.pivotAndShootHighFeed()).whileTrue(new PivotToAngleAndShoot(PivotConstants.kHighFeedAngle, IntakeConstants.kDesiredHighFeedMotorRPS, IntakeConstants.kDesiredHighFeedBeltSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kHighFeedTolerance));
    
    
    //TODO: add the trigger to pivot to backshot after merge

    //When the y button is held on op controller, the arm pivots and revs for amp shot.
    //Driver still has final say to make the shot
    new Trigger(() -> oi.pivotToAmp()).onTrue(new NewAutoPivotToAngle(PivotConstants.kAmpAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    new Trigger(() -> oi.revAmp()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredAmpMotorRPS, 0, INTAKE, LED, oi));
    
    //Driver has final say for speaker shots
    new Trigger(() -> oi.pivotToSubwooferShoot()).onTrue(new NewAutoPivotToAngle(PivotConstants.kSubwooferShootAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold, true));
    new Trigger(() -> oi.pivotToBackshots()).onTrue(new NewAutoPivotToAngle(PivotConstants.kOppositeSubwooferShootAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold, true));

    new Trigger(() -> oi.engageAutoShootSpinup()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredShootMotorRPS, 0, INTAKE, LED, oi));
  
    //new Trigger(() -> oi.testButton()).whileTrue(new PivotToAngleAndShoot(PivotConstants.kOppositeSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, IntakeConstants.kShootBeltMotorSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kPivotToleranceThreshold));
  }
  
    //autos that that we use Robot.java using the Sendable Chooser   
    //all the autos we use should have a method that returns them in robot container 
    public Command getDoNothingAuto(){
        return autoFactory.doNothing();
    }

    public Command getTestAuto(){
        return autoFactory.test();
    }
    public Command getTestSecondAuto(){
      return autoFactory.testTwo();
    }
    public Command getTestThreeAuto(){
      return autoFactory.testThree();
    }
    public Command getTestFourAuto(){
      return autoFactory.testFour();
    }
    public Command getTestFiveAuto(){
      return autoFactory.testFive();
    }

    public Command getMobilityStyleAuto(){
      return autoFactory.mobilityWithStyle();
    }

    public Command getPodiumSubwooferTwoNotesAuto(){
      return autoFactory.podiumSubwooferTwoNotes();
    }
    public Command getAmpSubwooferTwoNotesAuto(){
      return autoFactory.ampSubwooferTwoNotes();
    }
    public Command getMidSubwooferFourNotesAmpFirstAuto(){
      return autoFactory.midSubWooferFourNotesAmpFirst();
    }
    public Command getMidSubwooferFourNotesPodiumFirstAuto(){
      return autoFactory.midSubWooferFourNotesPodiumFirst();
    }

    public Command getShootAndStopAuto() {
      return autoFactory.shootAndStop();
    }  

}