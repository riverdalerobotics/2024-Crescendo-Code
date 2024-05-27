// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.R3P2CustomClasses.P2TalonFX;
import frc.robot.R3P2CustomClasses.TalonHelper;
import frc.robot.commands.intakeCommands.IntakeDefaultCommand;

/**
 * Contains the motors that power the arm fly wheels and index belt
 */
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  //There are 2 motors that power the intake flywheels
  P2TalonFX leftIntake;
  P2TalonFX rightIntake;

  MotionMagicVelocityVoltage motionVelV;
  double desiredRPS = 0;
  public boolean specCommandRunning = false;

  CANSparkMax belt;
  //CANcoder speedCoder;
  public IntakeSubsystem() {
 
    //TODO: create talon objects
    leftIntake = new P2TalonFX(Constants.IntakeConstants.kLeftIntakeMotorID);
    rightIntake = new P2TalonFX(Constants.IntakeConstants.kRightIntakeMotorID);
    //TODO: Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = TalonHelper.createTalonConfig(Constants.IntakeConstants.PIDConstants.kIntakeD, IntakeConstants.PIDConstants.kIntakeI, IntakeConstants.PIDConstants.kIntakeP, IntakeConstants.PIDConstants.kIntakeS, IntakeConstants.PIDConstants.kIntakeV, IntakeConstants.kFlywheelsGearRatio, IntakeConstants.kStatorCurrentLimit, IntakeConstants.peakDutyCycle);
    leftIntake.config(talonFXConfigs);
    rightIntake.config(talonFXConfigs);
    
    //TODO: set your follower motor
    rightIntake.setControl(new Follower(leftIntake.getDeviceID(), true));

    //We create a closedLoop controller and set the desired velocity to 0.
    //We can change the desired velocity whenever we choose to
    //TODO: create your closed loop controller

    motionVelV = new MotionMagicVelocityVoltage(0);

    //Talon motors allow for multiple PID values that can be swapped between during game use. We set our values on slot 0, so we make sure to tell our motor to refer to that slot
    //during usage
    //TODO: set your closedLoopController to Slot = 0
    
    motionVelV.Slot = 0;

    //{ControllerObject}.Slot = 0;
    
  }
  
 

  /**
   * Sets the Intake ControlMode to start moving towards the desired RPS
   * @param RPS desired RPS (rotations per second)
   */
  public void setIntakeVelocityRPS(double RPS) {
    leftIntake.setControl(motionVelV.withVelocity(RPS));
    desiredRPS = RPS;
    
  }
  public void setIntakeVelocityZero(double RPS) {
    leftIntake.setControl(motionVelV.withVelocity(0));
  }
 

  /**
   * Sets the tolerance used to check if the intake is at setpoint during closed loop control
   * @param tolerance
   */
  public void setIntakeTolerance(double tolerance) {
    leftIntake.setTolerance(tolerance);
    rightIntake.setTolerance(tolerance);
  }


  public void sendSmartDashboard() {
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sendSmartDashboard();
  }
}
