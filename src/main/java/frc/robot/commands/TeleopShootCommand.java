// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.intakeCommands.AutoRevFlywheelsIndefinitely;
import frc.robot.commands.intakeCommands.PowerBeltAndShooter;
import frc.robot.commands.pivotCommands.AutoPivotToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;


//READ THIS: https://docs.wpilib.org/en/2021/docs/software/commandbased/command-groups.html
//Useful doc for learning about different kinds of command groups

public class TeleopShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommandNew. */
  public TeleopShootCommand(PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED LED, OI oi) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new AutoPivotToAngle(PivotConstants.kSubwooferShootAngle, pivot), new AutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredShootMotorRPS, intake, LED)), new WaitForButton(() -> oi.shoot()), new PowerBeltAndShooter(intake, LED));
  }
}
