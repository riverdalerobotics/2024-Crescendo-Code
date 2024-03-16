// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.AutoPickUpCommand;
import frc.robot.commands.IntakeIndefinitelyCommand;
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commands.autonomousCommands.AutoPivotAndRevShooterCommand;
import frc.robot.commands.autonomousCommands.AutoPivotAndRevShooterIndefinitelyCommand;
import frc.robot.commands.autonomousCommands.AutoPivotAndShootCommand;
import frc.robot.commands.intakeCommands.AutoRevFlywheels;
import frc.robot.commands.intakeCommands.AutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.IntakeDefaultCommand;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.PowerBeltAndShooter;
import frc.robot.commands.pivotCommands.AutoPivotToAngle;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.commands.pivotCommands.PivotDefaultCommand;
import frc.robot.commands.pivotCommands.TuckCommand;
import frc.robot.commands.swerveCommands.AutoAlignWithNoteSwerve;
import frc.robot.commands.swerveCommands.AutoMoveToPredefined;
import frc.robot.commands.swerveCommands.SwerveDefaultCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
 
  
  ////private final SequentialCommandGroup PrepShotThenShoot = new SequentialCommandGroup(new ParallelCommandGroup(new AutoRevFlywheels(IntakeConstants.kDesiredShootMotorRPS, INTAKE, LED), new AutoPivotToAngle(PivotConstants.kSubwooferShootAngle, PIVOT)), new PowerBeltAndShooter(INTAKE, IntakeConstants.kDesiredShootMotorRPS, LED));

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    
    // Register Named Commands so that it can be used in the PathPlanning autos 
    //TODO: In Path Planner UI, remember to add the named commands 
    //Path planner is the software we use to make our autos
    NamedCommands.registerCommand("note error fix", new AutoAlignWithNoteSwerve(CHASSIS, oi, NOTE_LIMELIGHT, LED));
    NamedCommands.registerCommand("IntakeIndefinitely", new IntakeIndefinitelyCommand(PIVOT, INTAKE, LED));
    NamedCommands.registerCommand("AutoPivotAndRevShooterIndefinitely", new AutoPivotAndRevShooterIndefinitelyCommand(PIVOT, INTAKE, LED));
    NamedCommands.registerCommand("AutoPivotAndShoot", new AutoPivotAndShootCommand(PIVOT, INTAKE, LED));
    NamedCommands.registerCommand("RevToShootIndefinitely", new AutoRevFlywheelsIndefinitely( IntakeConstants.kDesiredShootMotorRPS, INTAKE, LED));



    CHASSIS.setDefaultCommand(new SwerveDefaultCommand (
      CHASSIS,
      oi,
      LED
    ));
  
    
    INTAKE.setDefaultCommand(new IntakeDefaultCommand(
      oi,
      INTAKE,
      LED
    ));
    
    PIVOT.setDefaultCommand(new PivotDefaultCommand(
      oi, 
      PIVOT
    ));
  }

  //Put triggers here that change the active commands
  //Triggers are conditions that activate commands
  //Triggers can activate as long as an input is true, or toggle on and off based on the input
  //For more information on triggers, see: https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html
  private void configureBindings() {

    //as long as the left trigger is held, note align will be active
    new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignWithNoteSwerve(CHASSIS, oi, NOTE_LIMELIGHT, LED));

    //as long as the right trigger is held, auto move to predefined will be active
    new Trigger(() -> oi.engageAutoMoveToPredefined()).whileTrue(new AutoMoveToPredefined(CHASSIS, oi, TAG_LIMELIGHT));
    

    //This command is unfinished, but the purpose is to rezero the arm if the encoder value is innacurate
    new Trigger(() -> oi.tuckArm1()).whileTrue(new TuckCommand(PIVOT));
    new Trigger(() -> oi.tuckArm2()).whileTrue(new TuckCommand(PIVOT));

    new Trigger(() -> oi.pivotToIntakePosition()).onTrue(new NewAutoPivotToAngle(PivotConstants.kIntakeAngle, PIVOT));
    new Trigger(() -> oi.pivotToSubwooferShoot()).onTrue(new NewAutoPivotToAngle(PivotConstants.kSubwooferShootAngle, PIVOT));
    new Trigger(() -> oi.pivotToFeed()).onTrue(new NewAutoPivotToAngle(PivotConstants.kFeedAngle, PIVOT));
      
    new Trigger(() -> oi.shootFeed()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredFeedMotorRPS, IntakeConstants.kDesiredFeedBeltSpeed, INTAKE, LED, oi));
    new Trigger(() -> oi.engageAutoShootSpinup()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredShootMotorRPS, 0, INTAKE, LED, oi));
    new Trigger(() -> oi.engageAutoIntakeSpinup()).whileTrue(new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredIntakeMotorRPS, IntakeConstants.kIntakeBeltMotorSpeed, INTAKE, LED, oi));
  }
  
    //autos that that we use Robot.java using the Sendable Chooser   
    //all the autos we use should have a method that returns them in robot container 
    public Command getDoNothingAuto(){
        return autoFactory.doNothing();
    }
    public Command getShootOnlyAuto(){
        return autoFactory.shootOnly();
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
    public Command getMobilityStyleAuto(){
      return autoFactory.mobilityWithStyle();
    }
    public Command getMobilityOutOfWay(){
      return autoFactory.outOfWayMobility();
    }
    public Command getWeirdPodiumSubwooferTwoNotesAuto(){
      return autoFactory.weirdPodiumSubwooferTwoNotes();
    }
    public Command getWeirdAmpSubwooferTwoNotesAuto(){
      return autoFactory.weirdAmpSubwooferTwoNotes();
    }
    public Command getWeirdMidSubwooferFourNotesAuto(){
      return autoFactory.weirdMidSubwooferTwoNotes();
    }
    public Command getShootAndStopAuto() {
      return autoFactory.shootAndStop();
    }  

}