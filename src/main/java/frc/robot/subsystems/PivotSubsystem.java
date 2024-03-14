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

    //These reset the motors to factory default every time the code runs
    pivot1.getConfigurator().apply(new TalonFXConfiguration());
    pivot2.getConfigurator().apply(new TalonFXConfiguration());



    //Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Config = talonFXConfigs.Slot0;
    slot0Config.kS = 0;
    //TODO: Find voltage required maintain position at the horizontal
    slot0Config.kV = 0;
    slot0Config.kP = PivotConstants.PIDConstants.kPivotP;
    slot0Config.kI = PivotConstants.PIDConstants.kPivotI;
    slot0Config.kD = PivotConstants.PIDConstants.kPivotD;

    //TODO: change the 0 encoder position of the arm to be horizontal with the ground

    //This tells the system that the controller is affecting an arm.
    //This changes how to feedforward values are applied to the system.
    //The highest voltage is needed when the arm is horizontal, and the lowest is needed when the arm is vertical
    //This voltage requirement follows a cosine ratio
    slot0Config.GravityType = GravityTypeValue.Arm_Cosine;


    //TODO: Figure out how to apply gear ratio conversion factor to the internal encoders (currently only measuring rotations when it should be degrees)

    /**Motion magic is a form of motion profiling offered by CTRE
    Read this page for more information on what motion profiling is: https://docs.wpilib.org/en/stable/docs/software/commandbased/profile-subsystems-commands.html
    In short, it gradually raises the desired setpoint, instead of abruptly changing the set point,
    resulting in a smoother motion with fewer voltage/current spikes */
    var motionMagicConfig = talonFXConfigs.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 0; //no limit on max velocity in degrees
    motionMagicConfig.MotionMagicAcceleration = 70; // limit of 160 degrees/s acceleration
    motionMagicConfig.MotionMagicJerk = 700; // limit of 700 degrees/s^2 jerk (limits acceleration)

    //Sets the current limit of our intake to ensure we don't explode our motors (which is bad)
    var currentConfig = talonFXConfigs.CurrentLimits;
    currentConfig.SupplyCurrentLimit = 60;
    currentConfig.SupplyCurrentLimitEnable = true;


    //This is not currently used, but may be useful for avoiding voltage spikes
    var closedLoopRampsConfig = talonFXConfigs.ClosedLoopRamps;
    closedLoopRampsConfig.VoltageClosedLoopRampPeriod = 0;
    
    

    //The motors are opposite to eachother, so one must be inverted
    pivot1.getConfigurator().apply(talonFXConfigs, 0.050);
    pivot2.getConfigurator().apply(talonFXConfigs, 0.050);
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
