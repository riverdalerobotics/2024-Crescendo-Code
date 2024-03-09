// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BlinkinLED;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {
  OI operatorInput;
  IntakeSubsystem intake;
  PIDController intakeSpeedController;
  BlinkinLED LED;
  double intakeSpeed = 10;
  double shootSpeed = 10;
  //half of setpoint is 0.008
  double kp = 0.0097;
  double ki = 0d;
  double kd = 0d;
  double tolerance = 3;
  boolean manual = false;

  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(OI opInput, IntakeSubsystem intake, BlinkinLED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorInput = opInput;
    this.intake = intake;
    this.LED = LED;
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

  //These lines were just for testing PID stuff. Keeping them here in case we want to use them again for testing
  //System.out.println(intakeSpeedController.getSetpoint());
  //System.out.println(intakeSpeedController.getPositionError());
  //System.out.println("INTAKE DEF WORKING");


     /**if(operatorInput.enableManualIntakeControl()){
       manual = true;
       intakeSpeedController.setSetpoint(0);
     }
    
    if(manual){
    intake.spinIntake(operatorInput.manualPowerIntake());
    }
    
    
    //These commands are still partially manual, but rely on determined PID values
    if (!manual){*/

    if (intakeSpeedController.atSetpoint() && intakeSpeedController.getSetpoint() != 0) {
      LED.disableFlywheelsRevvingLED();
      LED.enableFlywheelsReadyLED();
    }
    else if (intakeSpeedController.atSetpoint() == false && intakeSpeedController.getSetpoint() != 0) {
      LED.disableFlywheelsReadyLED();
      LED.enableFlywheelsRevvingLED();
    }
    else {
      LED.disableFlywheelsReadyLED();
      LED.disableFlywheelsRevvingLED();
    }
      
    intake.spinIntake(intakeSpeedController.calculate(intake.getSpeed()));
    
    if(operatorInput.engageAutoIntakeSpinup()){
      intakeSpeedController.setSetpoint(intakeSpeed*2);
      intake.spinBelt(IntakeConstants.kIntakeBeltMotorSpeed);
      //manual = false;
    }
    else if(operatorInput.engageAutoShootSpinup()){
      intakeSpeedController.setSetpoint(shootSpeed*2);
      //manual = false;
    }
    else {
      intakeSpeedController.setSetpoint(0);
    }

    /**
    if (intake.intakeCurrent()>0.085){
      intake.spinIntake(0);
      manual = true;
    }*/

    //During operation, the driver can hold down the right bumper to power the indexer to shoot
    if(operatorInput.shoot()) {
      intake.spinBelt(IntakeConstants.kShootBeltMotorSpeed);
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
