// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotToAngleAndRev extends ParallelDeadlineGroup {
  /** Creates a new PivotToAngleAndRev. */
  public PivotToAngleAndRev(double desiredAngle, double intakeRPS, PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED blinkin, OI oi, boolean considerGravity) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new NewAutoPivotToAngle(desiredAngle, pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold, considerGravity));
    addCommands(new NewAutoRevFlywheelsIndefinitely(intakeRPS, 0, intake, blinkin, oi));
  }

  /** An alternative constructor which allows us to set the pivot tolerance. This is useful when we want to call this method for angles that don't require a high accuracy, such as feed angles
  We sacrifice accuracy for speed as accuracy is irrelevant for these cases */
  public PivotToAngleAndRev(double desiredAngle, double intakeRPS, PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED blinkin, OI oi, boolean considerGravity, double pivotTolerance) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new NewAutoPivotToAngle(desiredAngle, pivot, pivotTolerance, considerGravity));
    addCommands(new NewAutoRevFlywheelsIndefinitely(intakeRPS, 0, intake, blinkin, oi));
  }
}
