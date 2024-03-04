// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {
  OI operatorInput;
  IntakeSubsystem intake;
  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(OI opInput, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorInput = opInput;
    this.intake = intake;
    addRequirements(intake);
    
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: this is for testing only
    intake.spinIntake(operatorInput.manShoot());
    intake.spinBelt(operatorInput.beltSpeed());
    

    //Intake will be active as long as the operator's x button is held down
    if(operatorInput.powerIntakeMechanisms()) {
      intake.engageIntake();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
