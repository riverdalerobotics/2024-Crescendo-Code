package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;


/**
 * This class is used to represent all the parts of an individual swerve module. It contains both the drive and turn motors of the swerve module. 
 * The purpose of creating a seperate class for the modules themselves is to allow them to be easily used in the chassis subsystem, as this class
 * contains multiple useful methods for manipulating the individual swerve module
 */
public class SwerveModule {
  /** Creates a new SwerveModuleSubsystem. */

  //These are actually the spark max controllers
  //Not the motors
  private final CANSparkMax mDriveMotor;
  private final CANSparkMax mTurnMotor;

  private final RelativeEncoder mDriveEncoder;
  private final AbsoluteEncoder mTurnEncoder;


  //These PID controllers are handled by the spark max motor controllers. This allows them to update more accurately and more frequently than the roborio
  private final SparkPIDController mTurnPIDController;
  private final SparkPIDController mDrivePIDController;



  //This variable tracks the state of the module
  private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());


  public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorInverted, boolean turnMotorInverted) {
    //All the motors we use are brushless (as far as I know)
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    

    //In the code below, we are changing the settings of the module's drive and turn motors to our desired settings. 

    //Ensures that default settings are set to a known state upon creation to avoid unknown changes remaining upon startup
    mDriveMotor.restoreFactoryDefaults();
    mTurnMotor.restoreFactoryDefaults();


    mDriveMotor.setInverted(driveMotorInverted);
    mTurnMotor.setInverted(turnMotorInverted);
    mDriveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    mTurnMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    
    mDriveEncoder = mDriveMotor.getEncoder();
    mTurnEncoder = mTurnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    mTurnPIDController = mTurnMotor.getPIDController();
    mTurnPIDController.setFeedbackDevice(mTurnEncoder);
    mDrivePIDController = mDriveMotor.getPIDController();
    mDrivePIDController.setFeedbackDevice(mDriveEncoder);


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


    //A wrapped PID controller means that the system the controller is affecting is circular.
    //We are telling it that the min and max positions are connected, allowing it to find smaller required rotations to the desired position
    mTurnPIDController.setPositionPIDWrappingEnabled(true);
    mTurnPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    mTurnPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);


    // Set the PID gains for the turning motor
    mTurnPIDController.setP(ModuleConstants.kTurningP);
    mTurnPIDController.setI(ModuleConstants.kTurningI);
    mTurnPIDController.setD(ModuleConstants.kTurningD);
    mTurnPIDController.setFF(ModuleConstants.kTurningFF);
    mTurnPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);
    mTurnPIDController.setSmartMotionAllowedClosedLoopError(ModuleConstants.kTurningTolerance, 0);


    //Sets the PID gains for the drive motor
    mDrivePIDController.setP(ModuleConstants.kDriveP);
    mDrivePIDController.setI(ModuleConstants.kDriveI);
    mDrivePIDController.setD(ModuleConstants.kDriveD);
    mDrivePIDController.setFF(ModuleConstants.kDriveFF);
    mDrivePIDController.setOutputRange(ModuleConstants.kDriveMinOutput, ModuleConstants.kDriveMaxOutput);


    mDriveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    mTurnMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);


    //Saves these settings to the modules. Useful in the event of our robot browning out (go to very low voltage), in which case this will save all the motor settings 
    mDriveMotor.burnFlash();
    mTurnMotor.burnFlash();

  }


  /**
   * Returns the current state of the module. The state of a module contains the current module rotation and velocity
   * 
   * @return the current angle and velocity (m/s) of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(mDriveEncoder.getVelocity(),
    new Rotation2d(mTurnEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module. The position of a module contains the current module rotation and distance travelled (found using encoder value)
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

  /**
   * Returns the velocity of the drive motor for the specific module
   * @return the velocity of the drive motor
   */
  public double getDriveVelocity() {
    return mDriveEncoder.getVelocity();
  }

  /**
   * Returns the velocity of the turning motor for the specific module
   * @return the velocity of the turn motor
   */
  public double getTurnVelocity() {
    return mTurnEncoder.getVelocity();
  }

  //Called on robot initialization to reset drive encoder positions
  public void resetEncoders() {
    mDriveEncoder.setPosition(0);
  }

  /**
   * Returns the module to a forward facing position and sets its speed to 0. This is used at the end of most swerve commands to clean up any motor values
   * and ensure the robot doesn't continue moving after the command has finished
   */
  public void setModuleZero() {

    //we create a new swerve module state with 0 rotation and 0 speed
    SwerveModuleState zero = new SwerveModuleState(0, new Rotation2d(0));
    //Finding the fasted way to reach zero angle
    zero = SwerveModuleState.optimize(zero, getState().angle);

    mDriveMotor.set(zero.speedMetersPerSecond);
    
    mTurnPIDController.setReference(zero.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = zero;
    }



  /**
   * Sets the module to a desired state. This state has already been desaturated by swerveChassisSubsystem.
   * This method sets the drive motor speed by dividing the desired speed by the physical max speed of the robot and passing that 
   * value directly into the motor controller as a percentage power.
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModulePosition().angle);
    SmartDashboard.putNumber("speed (m/s)", optimizedState.speedMetersPerSecond);

    mDriveMotor.set(optimizedState.speedMetersPerSecond / ChassisConstants.kPhysicalMaxSpeedMetersPerSecond);
    mTurnPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = optimizedState;
  }

  /**
   * Same function as the method above, but is used for cases where a speed of 0 is passed in, requiring the removeal of the deadband in the method above
   * @param desiredState desired state of the module
   */
  public void setDesiredStateNoDeadband(SwerveModuleState desiredState) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModulePosition().angle);

    mDriveMotor.set(optimizedState.speedMetersPerSecond / ChassisConstants.kPhysicalMaxSpeedMetersPerSecond);
    mTurnPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    mDesiredState = optimizedState;
  }

  /**
   * Sets the module to a desired state. This state has already been desaturated by swerveChassisSubsystem. 
   * This is an alternative setDesiredState method that uses a PID controller to handle the drive motor's velocity.
   * @param desiredState
   */
  public void setDesiredStatePIDDrive(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.005) {
      stop();
      return;
  }

  SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModulePosition().angle);
  SmartDashboard.putNumber("speed (m/s)", optimizedState.speedMetersPerSecond);

  mDrivePIDController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
  mTurnPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
}



  public SwerveModuleState getDesiredState() {
    return mDesiredState;
  }


  public double getTurnMotorVoltage() {
    return mTurnMotor.getBusVoltage();
  }

  //Stops the modules, but doesn't return the turn motor to forward facing position
  public void stop() {
    mDriveMotor.set(0);
    mTurnMotor.set(0);
  }



  public void setDriveCoast() {
    mDriveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setDriveBrake() {
    mDriveMotor.setIdleMode(IdleMode.kBrake);
  }
}
