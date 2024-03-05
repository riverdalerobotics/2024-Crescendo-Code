// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.AutoAlignWithNoteSwerve;
import frc.robot.commands.AutoPivotToAngle;
//import frc.robot.commands.AutoRevFlyWheels;
import frc.robot.commands.Autos;
import frc.robot.commands.PowerBeltAndShooter;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.defaultCommands.IntakeDefaultCommand;
import frc.robot.commands.defaultCommands.SwerveDefaultCommand;
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
  
  //private final IntakeSubsystem INTAKE = new IntakeSubsystem();
  
  OI oi = new OI();
  public final Limelight NOTE_LIMELIGHT = new Limelight("limelight-note");
  public final Limelight TAG_LIMELIGHT = new Limelight("limelight-tags"); 
  Autos autoFactory = new Autos();
  public final SwerveChassisSubsystem CHASSIS = new SwerveChassisSubsystem(oi);
  public final PivotSubsystem PIVOT = new PivotSubsystem();
  public final IntakeSubsystem INTAKE = new IntakeSubsystem();
  public final ClimberSubsystem CLIMB = new ClimberSubsystem();
 
  
  //private final SequentialCommandGroup PrepShotThenShoot = new SequentialCommandGroup(new ParallelCommandGroup(new AutoRevFlyWheels(IntakeConstants.kDesiredShootMotorRPS, INTAKE), new AutoPivotToAngle(PivotConstants.kSubwooferShootAngle, PIVOT)), new PowerBeltAndShooter(INTAKE, IntakeConstants.kDesiredShootMotorRPS));

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    // Register Named Commands so that it can be used in the PathPlanning autos 
    //TODO: In Path Planner UI, remember to add the named commands 

    // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    // NamedCommands.registerCommand("someOtherCommand", new IntakeDefaultCommand(oi, INTAKE));


    //Field reset toggle boost damp
    CHASSIS.setDefaultCommand(new SwerveDefaultCommand (
      CHASSIS,
      oi
    ));

    /**
    INTAKE.setDefaultCommand(new IntakeDefaultCommand(
      oi, 
      INTAKE));*/
  }

  //Put triggers here that change the active commands
  private void configureBindings() {

    new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignWithNoteSwerve(CHASSIS, () -> oi.xSpeed(), () -> oi.ySpeed(), () -> oi.rotate(), () -> oi.toggleFieldOriented(), NOTE_LIMELIGHT));
      
      }
  
   


    /** 
     * @return automonous period command
     */
    //autos that that we use Robot.java using the Sendable Chooser   
    public Command getPodiumSubwooferTwoNotesAuto(){
        return autoFactory.podiumSubwooferTwoNotes();
    }
    public Command getAmpSubwooferTwoNotesAuto(){
        return autoFactory.ampSubwooferTwoNotes();
    }
    public Command getMidSubwooferFourNotesAuto(){
        return autoFactory.midSubWooferFourNotes();
    }
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


    





      

}

