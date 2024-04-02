// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;




/* --------------------------------------------------------------------------- *
 * This command is used during autonomous. Its purpose is to be called         *
 * while moving towards a shooting position to prepare for the shot            *
 * and reduce the time spent waiting in the correct shooting position once     *
 * we are there                                                                *
 * --------------------------------------------------------------------------- */
public class PivotToAngleAndRevIndefinitely extends ParallelCommandGroup {
  /** Creates a new PivotToAngleAndRevIndefinitely. */
  public PivotToAngleAndRevIndefinitely(double desiredAngle, double intakeRPS, PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED blinkin, OI oi) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new NewAutoPivotToAngle(desiredAngle, pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold), new NewAutoRevFlywheelsIndefinitely(intakeRPS, 0, intake, blinkin, oi));
  }
}
