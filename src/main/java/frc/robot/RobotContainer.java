// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.defaultCommands.SwerveDefaultCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsytem;
import frc.robot.subsystems.SwerveChassisSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final SwerveChassisSubsystem CHASSIS = new SwerveChassisSubsystem(oi);
  public final static PivotSubsytem PIVOT = new PivotSubsytem();
  public final static IntakeSubsystem INTAKE = new IntakeSubsystem();
  public final static ClimberSubsystem CLIMB = new ClimberSubsystem();

  

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();



    //Field reset toggle boost damp
    CHASSIS.setDefaultCommand(new SwerveDefaultCommand (
      CHASSIS,
      () -> oi.xSpeed(),
      () -> oi.ySpeed(),
      () -> oi.rotate(),
      () -> oi.toggleFieldOriented(),
      () -> oi.toggleSlowMode()
    ));
  }

  //Put triggers here that change the active commands
  private void configureBindings() {
      
      }
  
   
   /** 
    * @return double
    */
   //TODO FIX THIS REMEMBER
  public double getAutonomousCommand() {
    //TODO: remove this
      int x = 0;
      return x;
  }
}
