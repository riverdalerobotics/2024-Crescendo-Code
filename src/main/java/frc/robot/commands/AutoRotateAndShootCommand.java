// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRotateAndShootCommand extends SequentialCommandGroup {
  /** Creates a new AutoRotateAndShootCommand. */
  public AutoRotateAndShootCommand(PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED LED) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new AutoPivotToAngle(PivotConstants.kSubwooferShootAngle, pivot), new AutoRevFlywheels(IntakeConstants.kDesiredShootMotorRPS, intake, LED)), new PowerBeltAndShooter(intake, LED));
  }
}
