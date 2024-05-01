package frc.robot.commands.intakeCommands;



// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * Revs the arm fly wheels to the desired rotaions per second. This command ends as soon as the desired
 * RPM is reached. The default command doesn't retain this RPS so this command should only be used
 * when the next command is continuing to power the flywheels
 */
public class NewAutoRevFlywheelsIndefinitely extends Command {
  /** Creates a new AutoRevFlyWheels. */
  BlinkinLED LED;
  public NewAutoRevFlywheelsIndefinitely(BlinkinLED LED) {
    this.LED = LED;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED.enableFlywheelsRevvingLED();
  }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
    
    //Once the desired velocity is set, the motor controller does all the work updating the speed of the motor so we don't have to
    }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
 }
