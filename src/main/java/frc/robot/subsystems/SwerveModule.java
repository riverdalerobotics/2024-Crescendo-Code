package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModuleSubsystem. */

  //These are actually the spark max controllers
  //Not the motors
  private final CANSparkMax mDriveMotor;
  private final CANSparkMax mTurnMotor;

  private final RelativeEncoder mDriveEncoder;
  private final AbsoluteEncoder mTurnEncoder;

  private final SparkPIDController mTurnPIDController;



  //This variable tracks the state of the module
  private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorInverted, boolean turnMotorInverted) {
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    
    //Ensures that default settings are a known state
    mDriveMotor.restoreFactoryDefaults();
    mTurnMotor.restoreFactoryDefaults();


    mDriveMotor.setInverted(driveMotorInverted);
    mTurnMotor.setInverted(turnMotorInverted);
    mDriveMotor.setSmartCurrentLimit(60);
    mTurnMotor.setSmartCurrentLimit(20);
    mDriveEncoder = mDriveMotor.getEncoder();
    mTurnEncoder = mTurnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    mTurnPIDController = mTurnMotor.getPIDController();
    mTurnPIDController.setFeedbackDevice(mTurnEncoder);


    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    mDriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    mDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);


    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    mTurnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    mTurnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    mTurnEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);


    mTurnPIDController.setPositionPIDWrappingEnabled(true);
    mTurnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    mTurnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);


    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    mTurnPIDController.setP(ModuleConstants.kTurningP);
    mTurnPIDController.setI(ModuleConstants.kTurningI);
    mTurnPIDController.setD(ModuleConstants.kTurningD);
    mTurnPIDController.setFF(ModuleConstants.kTurningFF);
    mTurnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);


    mDriveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    mTurnMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);


    mDriveMotor.burnFlash();
    mTurnMotor.burnFlash();

  }


  /**
   * Returns the current state of the module
   * 
   * @return the current angle and velocity (mps) of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(mDriveEncoder.getVelocity(),
    new Rotation2d(mTurnEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module
   *
   * @return distance travelled (meters) and current orientation
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(mDriveEncoder.getPosition(),
    new Rotation2d(mTurnEncoder.getPosition()));
  }

  
  /** 
   * @return double
   */
  public double getDrivePosition() {
    return mDriveEncoder.getPosition();
  }

  
  /** 
   * @return double
   */
  public double getTurningPosition() {
    return mTurnEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return mDriveEncoder.getVelocity();
  }

  public double getTurnVelocity() {
    return mTurnEncoder.getVelocity();
  }

  //Called on robot initialization to reset drive encoder positions
  public void resetEncoders() {
    mDriveEncoder.setPosition(0);
  }

  public void setModuleZero() {
    SwerveModuleState zero = new SwerveModuleState(0, new Rotation2d(0));
    //Finding the fasted way to reach zero angle
    zero = SwerveModuleState.optimize(zero, getState().angle);

    mDriveMotor.set(zero.speedMetersPerSecond);

    //Make sure that angle is returning correctly (angle may already be radians)
    mTurnPIDController.setReference(zero.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = zero;
    }



  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
    }

    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModulePosition().angle);
    SmartDashboard.putNumber("speed (m/s)", optimizedState.speedMetersPerSecond);

    mDriveMotor.set(optimizedState.speedMetersPerSecond / ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond);
    mTurnPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = optimizedState;
  }

  public SwerveModuleState getDesiredState() {
    return mDesiredState;
  }


  public double getTurnMotorVoltage() {
    return mTurnMotor.getBusVoltage();
  }

  public void stop() {
    mDriveMotor.set(0);
    mTurnMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
