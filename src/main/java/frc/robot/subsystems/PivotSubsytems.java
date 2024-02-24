// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class PivotSubsytems extends SubsystemBase {
  /** Creates a new pivotSubsytems. */
  TalonFX pivot1;
  TalonFX pivot2;
  static CANcoder pivotEncoder;
  
  public PivotSubsytems() {
    pivot1 = new TalonFX(10000);
    pivot2 = new TalonFX(10000);
    //pivot1.setNeutralMode(NeutralModeValue.Brake);
    pivotEncoder = new CANcoder(1000);
    
    
  }

  public void movePivot(double speed){
    pivot1.set(speed);
  }
  public void resetEncoders(){
    pivotEncoder.setPosition(0);
  }
  public double getEncoders(){
    StatusSignal<Double> pos = pivotEncoder.getPosition();
    double encoderPos = pos.getValue() * 0.14007201;
    return encoderPos;
  }
  public double getVoltage(){
    StatusSignal<Double> voltage = pivot1.getMotorVoltage();
    return voltage.getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
