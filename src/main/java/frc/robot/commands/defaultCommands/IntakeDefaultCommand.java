// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {
  OI operatorInput;
  IntakeSubsystem intake;
  PIDController intakeSpeedController;
  double intakeSpeed = 0d;
  double shootSpeed = 0d;
  double kp = 0d;
  double ki = 0d;
  double kd = 0d;
  double tolerance = 0d;
  boolean manual = false;

  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(OI opInput, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorInput = opInput;
    this.intake = intake;
    intakeSpeedController = new PIDController(kp, ki, kd);
    addRequirements(intake);
    
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSpeedController.setSetpoint(0);
    intakeSpeedController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  intake.spinIntake(operatorInput.manualShoot());
  System.out.println("INTAKE DEF WORKING");
  //intake.spinBelt(operatorInput.manualBeltSpeed());
  //TODO: this is for testing only
  // intake.spinIntake(operatorInput.manShoot());
  // intake.spinBelt(operatorInput.beltSpeed());
    // if(operatorInput.disableManualIntakeControl()){
    //   manual = true;
    //   intakeSpeedController.setSetpoint(0);
    // }
    
    // if(manual){
    // intake.spinIntake(operatorInput.manualShoot());
    // intake.spinBelt(operatorInput.manualBeltSpeed());
    // }
    
    // //This is the real thing...Perhaps (Brandon please look through this and make s| ure that this is whay u want)
    // if (!manual){
    //   intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
    // }
    // if(operatorInput.engageAutoIntakeSpinup()){
    //   intakeSpeedController.setSetpoint(intakeSpeed);
    //   manual = false;
    // }
    // else if(operatorInput.engageAutoShootSpinup()){
    //   intakeSpeedController.setSetpoint(shootSpeed);
    //   manual = false;
    // }
    



    // //Intake will be active as long as the operator's x button is held down
    // if(operatorInput.powerIntakeMechanisms()) {
    //   intake.engageIntake();

   // }
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
