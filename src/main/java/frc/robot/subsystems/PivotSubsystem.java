// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TalonHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;




/**
 * Handles the pivoting of the main arm mechanism
 */
public class PivotSubsystem extends SubsystemBase {
  /** Creates a new pivotSubsytems. */
  TalonFX pivot1;
  TalonFX pivot2;


  MotionMagicVoltage motionPositionVController;
  //TODO Check if this works
  double rotationToAngle = PivotConstants.kPivotEncoderRotationToDegrees;
  
  public PivotSubsystem() {
    pivot1 = new TalonFX(PivotConstants.kPivotMotor1ID);
    pivot2 = new TalonFX(PivotConstants.kPivotMotor2ID);

    //Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = TalonHelper.createTalonConfig(
      PivotConstants.PIDConstants.kPivotP,
      PivotConstants.PIDConstants.kPivotI,
      PivotConstants.PIDConstants.kPivotD,
      PivotConstants.PIDConstants.kPivotV,
      PivotConstants.PIDConstants.kPivotS,
      PivotConstants.PIDConstants.kMotionMagicCruiseVelocity,
      PivotConstants.PIDConstants.kMotionMagicAcceleration,
      PivotConstants.PIDConstants.kMotionMagicJerk,
      PivotConstants.kStatorCurrentLimit,
      0,
      PivotConstants.kPivotGearRatio,
      GravityTypeValue.Arm_Cosine
    );


    //TODO: Apply the velocity factor in the set velocity method
    //TODO: change the 0 encoder position of the arm to be horizontal with the ground
    //TODO: Figure out how to apply gear ratio conversion factor to the internal encoders (currently only measuring rotations when it should be degrees)
    

    //The motors are opposite to eachother, so one must be inverted
    TalonHelper.configTalon(pivot1, talonFXConfigs);
    TalonHelper.configTalon(pivot2, talonFXConfigs);
    pivot2.setControl(new StrictFollower(pivot1.getDeviceID()));

    //We create a closedLoop controller and set the desired velocity to 0.
    //We can change the desired velocity whenever we choose to
    motionPositionVController = new MotionMagicVoltage(0);
    //This ensures that we are using the PIDF configuration created above for slot 0
    motionPositionVController.Slot = 0;
    
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
