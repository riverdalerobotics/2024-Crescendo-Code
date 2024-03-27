// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combinationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.commands.intakeCommands.AutoRevAndBeltWhenReady;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveChassisSubsystem;


public class PivotToAngleAndShoot extends SequentialCommandGroup {
  /**
   * This command will pivot to an angle and rev simultaneously 
   * Once revved and in position, it will power the belt and fire the note

   * @param desiredAngle
   * @param intakeRPS
   * @param beltSpeed
   * @param swerve
   * @param pivot
   * @param intake
   * @param blinkin
   * @param oi
   * @param shootTime
   * @param considerGravity
   * @param pivotTolerance
   */
  public 
  PivotToAngleAndShoot(double desiredAngle, double intakeRPS, double beltSpeed, PivotSubsystem pivot, IntakeSubsystem intake, BlinkinLED blinkin, OI oi, double shootTime, boolean considerGravity, double pivotTolerance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PivotToAngleAndRev(desiredAngle, intakeRPS, pivot, intake, blinkin, oi, considerGravity, pivotTolerance), new AutoRevAndBeltWhenReady(intakeRPS, beltSpeed, intake, blinkin, oi, shootTime));
  }
}



