// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.commands.intakeCommands.AutoRevFlywheelsIndefinitely;
import frc.robot.commands.pivotCommands.AutoPivotToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotToAngleAndRev extends ParallelDeadlineGroup {
  /** Creates a new PivotToAngleAndRev. */
  public PivotToAngleAndRev(double desiredAngle, double intakeRPS, SwerveChassisSubsystem swerve, PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED blinkin, OI oi) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutoPivotToAngle(desiredAngle, pivot) );
    addCommands(new AutoRevFlywheelsIndefinitely(intakeRPS, 0, intake, blinkin, oi));
  }
}
