// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Contains the motors that power the arm fly wheels and index belt
 */
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  //There are 2 motors that power the intake flywheels
  TalonFX leftIntake;
  TalonFX rightIntake;

  CANSparkMax belt;
  //CANcoder speedCoder;
  public IntakeSubsystem() {
   // belt = new TalonFX(IntakeConstants.kBeltMotorID);
    belt = new CANSparkMax(IntakeConstants.kBeltMotorID, MotorType.kBrushless);
    leftIntake = new TalonFX(IntakeConstants.kLeftIntakeMotorID);
    rightIntake = new TalonFX(IntakeConstants.kRightIntakeMotorID);
    rightIntake.setInverted(true);
    
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
   * @return double
   */
  public double intakeVoltage(){
    StatusSignal<Double> voltage = leftIntake.getMotorVoltage();
    return voltage.getValue();
  }
  public double intakeCurrent(){
    StatusSignal<Double> current = leftIntake.getSupplyCurrent();
    return current.getValueAsDouble();
  }
  /**
   * Returns the speed of the intake in rotations per second
   * @return double
   */
  public double getSpeed(){
    StatusSignal<Double> speed = leftIntake.getVelocity();
    double speedD = speed.getValue();
    return speedD;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", getSpeed()*2);
    SmartDashboard.putNumber("Intake Current", intakeCurrent());
  }
}
