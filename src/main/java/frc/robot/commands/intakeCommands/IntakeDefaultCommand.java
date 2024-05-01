// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.HelperMethods;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;



//Remember, this class should have next to no functionality. All of the moving things should be done using other commands
//The only exception to this in crescendo code is that most commands allow a button to power the indexer for shooting


public class IntakeDefaultCommand extends Command {
  OI operatorInput;
  IntakeSubsystem intake;
  BlinkinLED LED;


  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.LED = LED;


    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DEF COMMAND STARTED");
  }


  //Liam: There should be very little happening in your default command execute method
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LED.disableFlywheelsReadyLED();
    LED.disableFlywheelsRevvingLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
