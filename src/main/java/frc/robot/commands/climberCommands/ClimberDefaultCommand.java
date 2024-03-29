// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberDefaultCommand extends Command {
  /** Creates a new ClimberDefaultCommand. */
  OI oi;
  ClimberSubsystem climber;
  
  public ClimberDefaultCommand(OI opInput, ClimberSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climb;
    addRequirements(climb);
    this.oi = opInput;
    
    
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(oi.Arm() >= 315 || oi.Arm() <= 45 && ClimbConstants.kMaxEncoderVal >= climber.getEncoder()){
      climber.climb(0.05);

    }else if (oi.Arm() >= 135 && oi.Arm() <= 225 && ClimbConstants.kMinEncoderVal <= climber.getEncoder() ){
      climber.climb(-0.05);
    }
    else{
      climber.climb(0);
    }


    if (oi.testButton()) {
      climber.climb(-0.05);
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
