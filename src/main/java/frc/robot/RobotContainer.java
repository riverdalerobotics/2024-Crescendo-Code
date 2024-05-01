// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

import frc.robot.commands.intakeCommands.AutoRevAndBeltWhenReady;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.NewIntakeDefaultCommand;
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
  public final IntakeSubsystem INTAKE = new IntakeSubsystem();
  //public final ClimberSubsystem CLIMB = new ClimberSubsystem();
 
  

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    
    INTAKE.setDefaultCommand(new NewIntakeDefaultCommand(
      oi,
      INTAKE,
      LED
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
  <p>
  --------------------------------------<p>
  Liam comment!!
  Triggers are the bread and butter of command based programming
  All the actual functionality of your subsytem should be handled through triggers activating commands
  Your default command should act as a base case that does nothing more than reset the system (intake) or hold a position (arm)
  <p>-------------------------------------*/
  private void configureBindings() {

    //as long as the right trigger is held, note align will be active
    
    //new Trigger(() -> oi.engageNoteAlignAssist()).whileTrue(new AutoAlignAndPickUp(CHASSIS, INTAKE, PIVOT, oi, LED, NOTE_LIMELIGHT));

    //as long as the left trigger is held, auto move to predefined will be active
    //new Trigger(() -> oi.engageAutoMoveToPredefined()).whileTrue(new AutoMoveToPredefined(CHASSIS, oi, TAG_LIMELIGHT));
    

    new Trigger(() -> oi.pivotToIntakePosition()).onTrue(new NewAutoPivotToAngle(PivotConstants.kIntakeAngle, PIVOT, PivotConstants.PIDConstants.kPivotToleranceThreshold));
  }
  
  


}