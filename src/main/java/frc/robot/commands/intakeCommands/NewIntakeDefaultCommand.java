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

public class NewIntakeDefaultCommand extends Command {
  OI operatorInput;
  IntakeSubsystem intake;
  BlinkinLED LED;
  double intakeSpeed = IntakeConstants.kDesiredIntakeMotorRPS;
  double shootSpeed = IntakeConstants.kDesiredShootMotorRPS;
  double tolerance = IntakeConstants.PIDConstants.kIntakeToleranceThreshold;
  boolean manual = false;

  /** Creates a new IntakeDefaultCommand. */
  public NewIntakeDefaultCommand(OI opInput, IntakeSubsystem intake, BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorInput = opInput;
    this.intake = intake;
    this.LED = LED;
    addRequirements(intake);

    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("DEF COMMAND STARTED");
    intake.setIntakeVelocityRPS(0);
    manual = false;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.specCommandRunning = false;


    //TODO: Fix manual as it currently messes with the acceleration ramping in PID control modes
    //makes it significantly slower
    
    /* 
     if(operatorInput.enableManualIntakeControl()){
        manual = true;
        intake.setIntakeVelocityRPS(0);
     }*/
    
    if(manual){
    intake.spinIntake(HelperMethods.applyInputDeadband(operatorInput.manualPowerIntake()));
    }
    
    //During operation, the driver can hold down the right bumper to power the indexer to shoot
    if(operatorInput.shoot()) {
      intake.spinBelt(IntakeConstants.kShootBeltMotorSpeed);
    }
    else{
      intake.spinBelt(0);
    }
    

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
