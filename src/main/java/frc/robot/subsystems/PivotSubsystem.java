// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;




/**
 * Handles the pivoting of the main arm mechanism
 */
public class PivotSubsystem extends SubsystemBase {
  /** Creates a new pivotSubsytems. */
  TalonFX pivot1;
  TalonFX pivot2;

  //TODO Check if this works
  double rotationToAngle = PivotConstants.kPivotEncoderRotationToDegrees;
  
  public PivotSubsystem() {
    pivot1 = new TalonFX(PivotConstants.kPivotMotor1ID);
    pivot2 = new TalonFX(PivotConstants.kPivotMotor2ID);

    
    
  }

  public void movePivot(double speed){
    pivot1.set(speed);
    pivot2.set(speed);
  }

  /**
   * Used when rezeroing the encoder position of the arm when it becomes inaccurate.
   * Sets the pivot encoder value to 0, representing the starting hard stop position
   */
  public void resetPivotEncoder(){
    pivot1.setPosition(0);
  }

  /**
   * Used to set a specific encoder angle
   * @param angle the desired angle
   */
  public void setPivotEncoder(double angle) {
    pivot1.setPosition(angle);
  }

  /**
   * Stops both pivot motors. Used when a current spike is detected, indicating
   * the pivot is pushing against a hard stop
   */
  public void stopAll() {
    pivot1.set(0);
    pivot2.set(0);
  }

  
  /** 
   * Returns the encoder value of the pivot in degrees.
   * Currently, 0 degrees refers to the position where the arm 
   * is resting on the hard stop inside frame perimeter (not intake hardstop)
   * @return double: angle of the pivot in degrees
   */
  public double getEncoders(){
    StatusSignal<Double> pos = pivot1.getPosition();
    double encoderPos = pos.getValue() * rotationToAngle;
    return encoderPos;
  }

  public double getVoltage(){
    StatusSignal<Double> voltage = pivot1.getMotorVoltage();
    return voltage.getValue();
  }

  public double getCurrent(){
    StatusSignal<Double> current = pivot1.getSupplyCurrent();
    return current.getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Voltage", getVoltage());
    SmartDashboard.putNumber("Pivot current", getCurrent());
    SmartDashboard.putNumber("Pivot position", getEncoders());
  }
}
