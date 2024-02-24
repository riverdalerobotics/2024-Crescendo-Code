// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  TalonFX leftIntake;
  TalonFX rightIntake;
  CANcoder speedCoder;
  public IntakeSubsystem() {
    leftIntake = new TalonFX(100000);
    rightIntake = new TalonFX(100000);
    rightIntake.setInverted(true);
    speedCoder = new CANcoder(1000);
    
  }
  public void spinIntake(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }
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
