// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public static CANSparkMax climbMotor;
  public ClimberSubsystem() {
    climbMotor = new CANSparkMax(0, MotorType.kBrushless);

  }


  public void climb(double speed){
    climbMotor.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
