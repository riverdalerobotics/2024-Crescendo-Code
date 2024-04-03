// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HelperMethods;
import frc.robot.TalonHelper;
import frc.robot.Constants.PivotConstants;
import frc.robot.R3P2CustomClasses.P2TalonFX;




/**
 * Handles the pivoting of the main arm mechanism
 */
public class PivotSubsystem extends SubsystemBase {
  /** Creates a new pivotSubsytems. */
  P2TalonFX pivot1;
  P2TalonFX pivot2;

  double maxPivotAngle = PivotConstants.PIDConstants.kMaxSetpoint;
  double minPivotAngle = PivotConstants.PIDConstants.kMinSetpoint;

  double desiredAngleDegrees = PivotConstants.PIDConstants.kMinSetpoint;
  MotionMagicVoltage motionPositionVController;
  //TODO Check if this works
  public boolean specCommandRunning = false;

  
  public PivotSubsystem() {
    pivot1 = new P2TalonFX(PivotConstants.kPivotMotor1ID);
    pivot2 = new P2TalonFX(PivotConstants.kPivotMotor2ID);

    //Create the configuration object that we will be using to apply our settings 
    //to both motors
    var talonFXConfigs = TalonHelper.createTalonConfig(
      PivotConstants.PIDConstants.kPivotP,
      PivotConstants.PIDConstants.kPivotI,
      PivotConstants.PIDConstants.kPivotD,
      PivotConstants.PIDConstants.kPivotV,
      PivotConstants.PIDConstants.kPivotS,
      PivotConstants.PIDConstants.kPivotG,
      PivotConstants.PIDConstants.kMotionMagicCruiseVelocity,
      PivotConstants.PIDConstants.kMotionMagicAcceleration,
      PivotConstants.PIDConstants.kMotionMagicJerk,
      PivotConstants.kStatorCurrentLimit,
      0,
      1/PivotConstants.kPivotGearRatio,
      PivotConstants.PIDConstants.kPivotPIDMaxOutput,
      GravityTypeValue.Arm_Cosine
    );


    //TODO: Apply the velocity factor in the set velocity method
    //TODO: change the 0 encoder position of the arm to be horizontal with the ground
    //TODO: Figure out how to apply gear ratio conversion factor to the internal encoders (currently only measuring rotations when it should be degrees)
    

    //The motors are opposite to eachother, so one must be inverted
    pivot1.config(talonFXConfigs);
    pivot2.config(talonFXConfigs);
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
   * Sets the pivot ControlMode to begin moving towards the desired angle in degress.
   * 0 degrees is horizontal facing towards the front of the robot. Positive angles move towards intake position. Negative angles move towards shoot position.
   * This method limits the degree value to be within the min and max range of the arm's movement
   * @param degrees
   */
  public void setPivotAngleDegrees(double degrees) {
    //ensure the pivot doesnt try to rotate outside of the max or min range
    degrees = HelperMethods.limitValInRange(minPivotAngle, maxPivotAngle, degrees);
    pivot1.setControl(motionPositionVController.withPosition(Units.degreesToRotations(degrees)));
    desiredAngleDegrees = degrees;
  }

  public void setPivotAngleDegreesNoLimit(double degrees) {
    pivot1.setControl(motionPositionVController.withPosition(Units.degreesToRotations(degrees)));
    desiredAngleDegrees = degrees;
  }



  /**
   * Used when rezeroing the encoder position of the arm when it becomes inaccurate.
   * Sets the pivot encoder value to 0, representing the starting hard stop position
   */
  public void resetPivotEncoder(){
    pivot1.setPosition(Units.degreesToRotations(PivotConstants.kZeroAngle));
  }

  //TODO: set the arm angle to the min position on robot start
  /**
   * Used to set a specific encoder angle
   * @param angle the desired angle in degrees
   */
  public void setPivotEncoder(double angle) {
    pivot1.setPosition(Units.degreesToRotations(angle));
    desiredAngleDegrees = angle;
  }

  /**
   * Stops both pivot motors. Used when a current spike is detected, indicating
   * the pivot is pushing against a hard stop
   */
  public void stopAll() {
    pivot1.set(0);
    pivot2.set(0);
  }

  
  //TODO: Change this javadoc to explain new angle measurements
  /** 
   * Returns the encoder value of the pivot in degrees.
   * Currently, 0 degrees refers to the point of arm rotation where it requires 
   * the most power to hold it in place
   * @return double: angle of the pivot in degrees
   */
  public double getEncoders(){
    StatusSignal<Double> pos = pivot1.getPosition();
    double encoderPos = Units.rotationsToDegrees(pos.getValue());
    return encoderPos;
  }


  /**
   * Returns the encoder value of the pivot in rotations.
   * This value is affected by the pivot's specified gear ratio
   * @return double: number of rotations of the pivot
   */
  public double getRotation() {
    return pivot1.getPosition().getValueAsDouble();
  }

  public double getVoltage(){
    StatusSignal<Double> voltage = pivot1.getMotorVoltage();
    return voltage.getValue();
  }

  /** */
  public double getCurrent(){
    StatusSignal<Double> current = pivot1.getSupplyCurrent();
    return current.getValueAsDouble();
  }


  public P2TalonFX getPivot1() {
    return pivot1;
  }

  /**
   * Sets the tolerance used to check if the arm is at setpoint during closed loop control
   * @param tolerance
   */
  public void setPivotTolerance(double tolerance) {
    pivot1.setTolerance(tolerance);
    pivot2.setTolerance(tolerance);

  }


  public void sendSmartDashboard() {
    SmartDashboard.putNumber("Pivot/Pivot Voltage", getVoltage());
    SmartDashboard.putNumber("Pivot/Pivot Current", getCurrent());
    SmartDashboard.putNumber("Pivot/Pivot Position", getEncoders());
    SmartDashboard.putNumber("Pivot/Pivot rotation", getRotation());
    SmartDashboard.putBoolean("Pivot/Auto Piv", specCommandRunning);
  }

  @Override
  public void periodic() {
    //System.out.println(motionPositionVController.Position);
    //System.out.println("acc positoon" + getRotation());
    // This method will be called once per scheduler run
    sendSmartDashboard();
  }
}
