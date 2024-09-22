// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.R3P2CustomClasses.P2TalonFX;
import frc.robot.R3P2CustomClasses.TalonHelper;

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

  //TODO: Use motion magic built into the talons to make smoother rev up that doesn't draw 5 volts
  CANSparkMax belt;
  //CANcoder speedCoder;
  public IntakeSubsystem() {

    //Leader motor
    leftIntake = new P2TalonFX(IntakeConstants.kLeftIntakeMotorID);

    //Follower motor
    rightIntake = new P2TalonFX(IntakeConstants.kRightIntakeMotorID);

    belt = new CANSparkMax(IntakeConstants.kBeltMotorID, MotorType.kBrushless);


    //Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = TalonHelper.createTalonConfig(
      IntakeConstants.PIDConstants.kIntakeP,
      IntakeConstants.PIDConstants.kIntakeI,
      IntakeConstants.PIDConstants.kIntakeD,
      IntakeConstants.PIDConstants.kIntakeV,
      IntakeConstants.PIDConstants.kIntakeS,
      0,
      IntakeConstants.PIDConstants.kMotionMagicCruiseVelocity,
      IntakeConstants.PIDConstants.kMotionMagicAcceleration,
      IntakeConstants.PIDConstants.kMotionMagicJerk,
      IntakeConstants.kStatorCurrentLimit,
      0,
      1/IntakeConstants.kFlywheelsGearRatio,
      IntakeConstants.PIDConstants.kIntakePIDMaxOutput
    );

    //The motors are opposite to eachother, so one must be inverted
    leftIntake.config(talonFXConfigs);
    rightIntake.config(talonFXConfigs);
    rightIntake.setControl(new Follower(leftIntake.getDeviceID(), true));

    //We create a closedLoop controller and set the desired velocity to 0.
    //We can change the desired velocity whenever we choose to
    motionVelV = new MotionMagicVelocityVoltage(0);

    //This ensures that we are using the PIDF configuration created above for slot 0
    //Talon motors allow for multiple PID values that can be swapped between during game use. We set our values on slot 0, so we make sure to tell our motor to refer to that slot
    //during usage
    motionVelV.Slot = 0;
    
  }
  
  /** 
   * Sets intake speed, can be used as a form of manual intake control
   * @param speed
   */
  public void spinIntake(double speed){
    leftIntake.set(speed);
    rightIntake.set(-speed);
  }

  /**
   * Sets the Intake ControlMode to start moving towards the desired RPS
   * @param RPS desired RPS (rotations per second)
   */
  public void setIntakeVelocityRPS(double RPS) {
    //Right will be powered as well because it is set to follow 
    leftIntake.setControl(motionVelV.withVelocity(RPS));
    desiredRPS = RPS;
  }
  
  /** 
   * Spins the indexer belt. Positive values spin inwards. Negative values spin outwards
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
  /**
   * Returns the value of current being supplied to the spark max motor controller
   * @return Supply current. This value will always be positive regardless of motor direction
   */
  public double flywheelSupplyCurrent(){
    StatusSignal<Double> current = leftIntake.getSupplyCurrent();
    return current.getValueAsDouble();
  }

  /**
   * Returns the value of current being supplied to the motor 
   * @return Torque current. This value will always be positive regardless of motor direction
   */
  public double flywheelTorqueCurrent() {
    return leftIntake.getTorqueCurrent().getValueAsDouble();
  }
  /**
   * Returns the speed of the intake in rotations per second
   * @return double
   */
  public double getSpeed(){
    StatusSignal<Double> flyWheelVelocity = leftIntake.getVelocity();
    //This returns in rotations per second
    return flyWheelVelocity.getValue();
  }


  public double getRotations() {
    return leftIntake.getPosition().getValueAsDouble();
  }

  public P2TalonFX getLeftIntakeMotor() {
    return leftIntake;
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
    SmartDashboard.putNumber("Intake/Intake Speed", getSpeed());
    SmartDashboard.putNumber("Intake/Intake Rotations", getRotations());
    SmartDashboard.putNumber("Intake/Intake Torque Current", flywheelTorqueCurrent());
    SmartDashboard.putNumber("Intake/Intake Supply Current", flywheelSupplyCurrent());
    SmartDashboard.putBoolean("Intake/Is command active", specCommandRunning);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sendSmartDashboard();
  }
}
