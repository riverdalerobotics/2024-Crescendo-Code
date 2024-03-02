// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX leftIntake;
  TalonFX rightIntake;
  TalonFX belt;
  CANcoder speedCoder;
  public IntakeSubsystem() {
    belt = new TalonFX(IntakeConstants.kBeltMotorID);
    leftIntake = new TalonFX(IntakeConstants.kLeftIntakeMotorID);
    rightIntake = new TalonFX(IntakeConstants.kRightIntakeMotorID);
    rightIntake.setInverted(true);
    speedCoder = new CANcoder(IntakeConstants.kSpeedEncoderID);
    
  }
  
  /** 
   * @param speed
   */
  public void spinIntake(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }
  
  /** 
   * @param speed
   */
  public void spinBelt(double speed){
    belt.set(speed);
  }

  /**
   * Powers the intake and belt motors at desired speeds for intaking notes
   */
  public void engageIntake() {
    spinIntake(-0.1);
    spinBelt(-0.1);
  }


  
  /** 
   * @return double
   */
  public double intakeVoltage(){
    StatusSignal<Double> voltage = leftIntake.getMotorVoltage();
    return voltage.getValue();
  }
  
  /**
   * Returns the speed of the intake in rotations per second
   * @return double
   */
  public double getSpeed(){
    StatusSignal<Double> speed = speedCoder.getVelocity();
    double speedD = speed.getValue();
    return speedD;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
