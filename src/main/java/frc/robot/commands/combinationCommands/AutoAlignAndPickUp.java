// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.Limelight;
import frc.robot.OI;
import frc.robot.commands.swerveCommands.AutoAlignWithNoteSwerve;
import frc.robot.commands.swerveCommands.DriveXMetersForward;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignAndPickUp extends SequentialCommandGroup {
  /** Creates a new AutoAlignAndPickUp. */
  public AutoAlignAndPickUp(SwerveChassisSubsystem swerve, IntakeSubsystem intake, PivotSubsystem pivot, OI oi, BlinkinLED LED, Limelight noteLimelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoAlignWithNoteSwerve(swerve, oi, noteLimelight, LED), new ParallelDeadlineGroup(new DriveXMetersForward(swerve, 0.8), new IntakeIndefinitelyCommand(pivot, intake, LED, oi)));
  }
}
