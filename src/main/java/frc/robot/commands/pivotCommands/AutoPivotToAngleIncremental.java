// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPivotToAngleIncremental extends SequentialCommandGroup {
  /** Creates a new AutoPivotToAngleIncremental. */
  PivotSubsystem pivot;
  public AutoPivotToAngleIncremental(PivotSubsystem pivot, double desiredAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new NewAutoPivotToAngle(-90, pivot, 10), new NewAutoPivotToAngle(desiredAngle, pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold));
  }
}
