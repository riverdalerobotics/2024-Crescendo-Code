// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TalonHelper;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

/**
 * Contains the motors that power the arm fly wheels and index belt
 */
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  //There are 2 motors that power the intake flywheels
  TalonFX leftIntake;
  TalonFX rightIntake;

  MotionMagicVelocityVoltage motionVelV;

  //TODO: Use motion magic built   into the talons to make smoother rev up that doesn't draw 5 volts
  CANSparkMax belt;
  //CANcoder speedCoder;
  public IntakeSubsystem() {
   // belt = new TalonFX(IntakeConstants.kBeltMotorID);
    belt = new CANSparkMax(IntakeConstants.kBeltMotorID, MotorType.kBrushless);

    //Leader motor
    leftIntake = new TalonFX(IntakeConstants.kLeftIntakeMotorID);

    //Follower motor
    rightIntake = new TalonFX(IntakeConstants.kRightIntakeMotorID);

    //These reset the motors to factory default every time the code runs
    leftIntake.getConfigurator().apply(new TalonFXConfiguration());
    rightIntake.getConfigurator().apply(new TalonFXConfiguration());


    //TODO: Figure out how to apply gear ration conversion factor to the internal encoder
    //Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = TalonHelper.createTalonConfig(
      IntakeConstants.PIDConstants.kIntakeP,
      IntakeConstants.PIDConstants.kIntakeI,
      IntakeConstants.PIDConstants.kIntakeD,
      IntakeConstants.PIDConstants.kIntakeV,
      IntakeConstants.PIDConstants.kIntakeS,
      IntakeConstants.PIDConstants.kMotionMagicCruiseVelocity,
      IntakeConstants.PIDConstants.kMotionMagicAcceleration,
      IntakeConstants.PIDConstants.kMotionMagicJerk,
      IntakeConstants.kStatorCurrentLimit,
      0,
      IntakeConstants.kFlywheelsGearRatio
    );

    //TODO: Find voltage required to spin at 1 RPS. This value is multipled by the requested speed
    //TODO: Apply the velocity factor in the set velocity method

    //The motors are opposite to eachother, so one must be inverted
    TalonHelper.configTalon(leftIntake, talonFXConfigs);
    TalonHelper.configTalon(rightIntake, talonFXConfigs);
    rightIntake.setControl(new StrictFollower(leftIntake.getDeviceID()));
    rightIntake.setInverted(true);

    //We create a closedLoop controller and set the desired velocity to 0.
    //We can change the desired velocity whenever we choose to
    motionVelV = new MotionMagicVelocityVoltage(0);
    //This ensures that we are using the PIDF configuration created above for slot 0
    motionVelV.Slot = 0;
  }
  
  /** 
   * @param speed
   */
  public void spinIntake(double speed){
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void setIntakeVelocityRPS(double RPS) {
    //Right will be powered as well because it is set to follow 
    leftIntake.setControl(motionVelV.withVelocity(RPS));
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
    var rotorVelocitySignal = leftIntake.getRotorVelocity();
    //This returns in rotations per second
    return rotorVelocitySignal.getValue();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", getSpeed()*2);
    SmartDashboard.putNumber("Intake Current", intakeCurrent());
  }
}
