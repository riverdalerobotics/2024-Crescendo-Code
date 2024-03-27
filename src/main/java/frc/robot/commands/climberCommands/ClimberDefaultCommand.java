// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */
  OI oi;
  ClimberSubsystem climb;
  
  public ClimberDefaultCommand(OI opInput, ClimberSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    addRequirements(climb);
    this.oi = opInput;
    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(oi.Arm() == 0 ){
      climb.climb(0.2);
    }else if (oi.Arm() == 180){
      climb.climb(-0.2);
    }
    else{
      climb.climb(0);
    }
    
  
  }
  // 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
