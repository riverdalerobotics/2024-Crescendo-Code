// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public CANSparkMax climbMotor;
  public RelativeEncoder climbEncoder;
  
  public ClimberSubsystem() {
    climbMotor = new CANSparkMax(0, MotorType.kBrushless);
    climbEncoder = climbMotor.getEncoder();

  }

  public  double getEncoder(){
    return climbEncoder.getPosition();

  }
  public void climb(double speed){
    climbMotor.set(speed);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb/ClimbEncoder", getEncoder());
    // This method will be called once per scheduler run
  }
}
