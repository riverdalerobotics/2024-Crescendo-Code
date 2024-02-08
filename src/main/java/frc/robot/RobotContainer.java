// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoShootCommand;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final SwerveChassisSubsystem CHASSIS = new SwerveChassisSubsystem();
  // Create our inputs
  //XboxController intakeController = new XboxController(0);
  //XboxController moveController = new XboxController(1);
  OI oi = new OI();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();



    //Field reset toggle boost damp
    swerveSubsystem.setDefaultCommand(new SwerveDefaultCommand (
      swerveSubsystem,
      () -> OI.xSpeed(),
      () -> OI.ySpeed(),
      () -> OI.rotate(),
      () -> OI.toggleFieldOriented(),
      () -> OI.toggleSlowMode()
    ));
  }

  private void configureBindings() {
      
      }
  
   //TODO FIX THIS REMEMBER
  public double getAutonomousCommand() {
    //TODO: remove this
      int x = 0;
      return x;
  }
}
