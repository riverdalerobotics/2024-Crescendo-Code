// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.IntakeConstants.PIDConstants;
import frc.robot.commands.climberCommands.ClimberDefaultCommand;
import frc.robot.commands.combinationCommands.AutoAlignAndPickUp;
import frc.robot.commands.combinationCommands.IntakeIndefinitelyCommand;
import frc.robot.commands.combinationCommands.PivotToAngleAndRevIndefinitely;
import frc.robot.commands.combinationCommands.PivotToAngleAndShoot;
import frc.robot.commands.intakeCommands.AutoRevAndBeltWhenReady;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.NewIntakeDefaultCommand;
import frc.robot.commands.pivotCommands.AprilTagPivot;
import frc.robot.commands.pivotCommands.AutoShootFromPredefinedDistance;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.commands.pivotCommands.NewPivotDefaultCommand;
import frc.robot.commands.swerveCommands.AutoAlignWithNoteSwerve;
import frc.robot.commands.swerveCommands.AutoFaceSpeakerCommand;
import frc.robot.commands.swerveCommands.HandSignalSwerveCommand;
import frc.robot.commands.swerveCommands.RotateToShuttle;
import frc.robot.commands.swerveCommands.SwerveDefaultCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

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
  //public final ClimberSubsystem CLIMB = new ClimberSubsystem();
 
  

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

    NamedCommands.registerCommand("PivotAndRevForShotBack", new PivotToAngleAndRevIndefinitely(PivotConstants.kOppositeSubwooferShootAngle, 71, PIVOT, INTAKE, LED, oi));
    NamedCommands.registerCommand("PivotAndShootBack", new PivotToAngleAndShoot(PivotConstants.kOppositeSubwooferShootAngle, 71, IntakeConstants.kShootBeltMotorSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    NamedCommands.registerCommand("ShootFromCurrentAngle", new AutoRevAndBeltWhenReady(IntakeConstants.kDesiredShootMotorRPS, IntakeConstants.kShootBeltMotorSpeed, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded));
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


    //CLIMB.setDefaultCommand(new ClimberDefaultCommand(oi, CLIMB));
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
    
    
    //TODO: uncomment this
    //new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignAndPickUp(CHASSIS, INTAKE, PIVOT, oi, LED, NOTE_LIMELIGHT));

    //as long as the left trigger is held, auto move to predefined will be active
    //new Trigger(() -> oi.engageAutoMoveToPredefined()).whileTrue(new AutoMoveToPredefined(CHASSIS, oi, TAG_LIMELIGHT));
    

    //This command is unfinished, but the purpose is to rezero the arm if the encoder value is innaccurate
    //new Trigger(() -> oi.tuckArm1()).whileTrue(new NewAutoPivotToAngle(PivotConstants.kZeroAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    //new Trigger(() -> oi.tuckArm2()).whileTrue(new NewAutoPivotToAngle(PivotConstants.kZeroAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));

    
    new Trigger(() -> oi.pivotToIntakePosition()).onTrue(new NewAutoPivotToAngle(PivotConstants.kIntakeAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    new Trigger(() -> oi.engageIntake()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredIntakeMotorRPS, IntakeConstants.kIntakeBeltMotorSpeed, INTAKE, LED, oi));
    //TODO: __________
    // new Trigger(() -> oi.pivotToIntakePosition()).onFalse(new NewAutoPivotToAngle(PivotConstants.PIDConstants.kMinSetpoint+1, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    
     //new Trigger(() -> oi.spinIntake()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredIntakeMotorRPS, IntakeConstants.kIntakeBeltMotorSpeed, INTAKE, LED, oi));

    //Feed controls do not require driver to press the fire button.
    //They will shoot as soon as intake and angle are prepared
    new Trigger(() -> oi.pivotAndShootLowFeed()).whileTrue(new PivotToAngleAndShoot(PivotConstants.kFeedAngle, IntakeConstants.kDesiredFeedMotorRPS, IntakeConstants.kDesiredFeedBeltSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, false, PivotConstants.PIDConstants.kLowFeedTolerance));
    new Trigger(() -> oi.pivotToHighFeed()).onTrue(new NewAutoPivotToAngle(PivotConstants.kHighFeedAngle, PIVOT, PivotConstants.PIDConstants.kHighFeedTolerance));
    new Trigger(() -> oi.revHighFeed()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredHighFeedMotorRPS, 0, INTAKE, LED, oi));
    //new Trigger(() -> oi.pivotFromfar()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredHighFeedMotorRPS, 0, INTAKE, LED, oi));
    //When the y button is held on op controller, the arm pivots and revs for amp shot.
    //Driver still has final say to make the shot
    new Trigger(() -> oi.pivotToAmp()).onTrue(new NewAutoPivotToAngle(PivotConstants.kAmpAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    new Trigger(() -> oi.revAmp()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredAmpMotorRPS, 0, INTAKE, LED, oi));
   
    // pivots to the angle when given a distance
    //new Trigger(() -> oi.pivotFromfar()).onTrue(new AutoShootFromPredefinedDistance(PIVOT, 8));
    
    //Driver has final say for speaker shots
    new Trigger(() -> oi.pivotToSubwooferShoot()).onTrue(new NewAutoPivotToAngle(PivotConstants.kSubwooferShootAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold, true));
    new Trigger(() -> oi.pivotToReverseShot()).onTrue(new NewAutoPivotToAngle(PivotConstants.kOppositeSubwooferShootAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold, true));

    new Trigger(() -> oi.engageAutoShootSpinup()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredShootMotorRPS, 0, INTAKE, LED, oi));
  
    //new Trigger(() -> oi.testButton()).whileTrue(new PivotToAngleAndShoot(PivotConstants.kOppositeSubwooferShootAngle, IntakeConstants.kDesiredShootMotorRPS, IntakeConstants.kShootBeltMotorSpeed, PIVOT, INTAKE, LED, oi, IntakeConstants.kShootTimeNeeded, true, PivotConstants.PIDConstants.kPivotToleranceThreshold));
    //new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignWithNoteSwerve(CHASSIS, oi, NOTE_LIMELIGHT, LED));
   
    //offseason test
    //new Trigger(() -> oi.handSignalMove()).whileTrue(new HandSignalSwerveCommand(CHASSIS, NOTE_LIMELIGHT));
    new Trigger(() -> oi.rotateRobot()).whileTrue(new RotateToShuttle(CHASSIS, oi, LED));
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

    public Command getWindsorLeaveAfterShootingPodiumSideAuto(){
      return autoFactory.leaveAfterShootingPodiumSide();
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

    public Command getMidSubwooferFourNotesActualAuto(){
      return autoFactory.midSubwooferFourNotesActual();
    }

    public Command getShootAndStopAuto() {
      return autoFactory.shootAndStop();
    }  

    
    public Command getQAOneAuto() {
      return autoFactory.queenAOne();
    }  
    
    public Command getQATwoAuto() {
      return autoFactory.queenATwo();
    }  
    
    public Command getQAThreeAuto() {
      return autoFactory.queenAThree();
    }  
    
    public Command getQAFourAuto() {
      return autoFactory.queenAFour();
    }  


}