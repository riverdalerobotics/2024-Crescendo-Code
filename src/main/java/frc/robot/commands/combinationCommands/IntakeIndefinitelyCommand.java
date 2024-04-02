// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.intakeCommands.NewAutoRevFlywheelsIndefinitely;
import frc.robot.commands.pivotCommands.NewAutoPivotToAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * This command is similar to the PickUp command, with the main difference being that it has no 
 * end condition. Unless interrupted, this command will run intake indefinitely.
 * The purpose of this command is use during autonomous, as well as use for trigger commands
 */

public class IntakeIndefinitelyCommand extends ParallelCommandGroup {
  /** Creates a new IntakeIndefinitelyCommandNew. */
  public IntakeIndefinitelyCommand(PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED LED, OI oi) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new NewAutoPivotToAngle(PivotConstants.kIntakeAngle, pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold), new NewAutoRevFlywheelsIndefinitely(IntakeConstants.kDesiredIntakeMotorRPS, IntakeConstants.kIntakeBeltMotorSpeed, intake, LED, oi));
  }
}
