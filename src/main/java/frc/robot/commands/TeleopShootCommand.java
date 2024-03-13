// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopShootCommand extends SequentialCommandGroup {
  /** Creates a new ShootCommandNew. */
  public TeleopShootCommand(PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED LED, OI oi) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelDeadlineGroup(new AutoPivotToAngle(PivotConstants.kSubwooferShootAngle, pivot), new AutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredShootMotorRPS, intake, LED)), new WaitForButton(() -> oi.shoot()), new PowerBeltAndShooter(intake, LED));
  }
}
