// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//SWERVE MAINTENANCE TO DO LIST (FOR YOU BRANDON):

//1. Re-zero turn motor's absolute encoders after every game (put them into the physical zero then reset offset with rev hardware client)(get jack to help you rezero)
//2. If speeds are getting out of wack, check what speeds the swerveModuleStates are requesting

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BlinkinLED;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.PathPlannerConstants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;


public class SwerveChassisSubsystem extends SubsystemBase {
  
  public boolean commandActive = false;

  //we create 4 swerve module objects representing the 4 swerve modules on our chassis
  //see SwerveModule.java for more information on these objects as that is where we create the class

  private final SwerveModule frontLeft = new SwerveModule(ChassisConstants.kFrontLeftDriveMotorPort, 
  ChassisConstants.kFrontLeftTurningMotorPort, 
  ChassisConstants.kFrontLeftDriveEncoderReversed, 
  ChassisConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(ChassisConstants.kFrontRightDriveMotorPort,
  ChassisConstants.kFrontRightTurningMotorPort,
  ChassisConstants.kFrontRightDriveEncoderReversed,
  ChassisConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(ChassisConstants.kBackLeftDriveMotorPort,
  ChassisConstants.kBackLeftTurningMotorPort,
  ChassisConstants.kBackLeftDriveEncoderReversed,
  ChassisConstants.kBackLeftTurningEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(ChassisConstants.kBackRightDriveMotorPort,
  ChassisConstants.kBackRightTurningMotorPort,
  ChassisConstants.kBackRightDriveEncoderReversed,
  ChassisConstants.kBackRightTurningEncoderReversed);

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
  private double maxTeleopDriveSpeed;
  private double maxTeleopAngularSpeed;
  private boolean isFieldOriented;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(ChassisConstants.kDriveKinematics, getRotation2d(), getSwerveModulePositions(), new Pose2d(0, 0, getRotation2d()));

  BlinkinLED LED;
  /** Creates a new SwerveChassisSubsystem. */
  public SwerveChassisSubsystem(BlinkinLED LED) {
    this.maxTeleopDriveSpeed = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
    this.maxTeleopAngularSpeed = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    //slew rate limiters are meant to limit acceleration to avoid sudden speed increases that can damage motors
    //they create smoother motion with gradual acceleration
    this.xLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond);
    this.yLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond);
    this.turnLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecond);
    resetModules();
    this.isFieldOriented = false;
    this.LED = LED;



    //This resets the gyro heading when the robot is turned on after a short delay.
    //The reason we need a delay is to give the gyroscope time to calibrate, as if we did it instantly, the value may become innacurate
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
        gyro.setAngleAdjustment(180);
    } catch (Exception e) {
    }
  }).start();

  //IF YOU ARE ROOKIE READING THIS CODE: Please ignore this code as it is for path planner and you don't need to learn about that yet
  
  //Pathplanner Stuff
  AutoBuilder.configureHolonomic( //Configures pathfinder with basic constraints and functionality of robot
    this::getPose, // Robot pose supplier (Pose2d)
    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose) (Pose2d)
    this::getVelocities,  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    this::driveSwerve,  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    new HolonomicPathFollowerConfig( //Object with the configurations for our drive train, particularly max speeds, PID Constants, and radius of our base
      PathPlannerConstants.TRANSLATION_PID_CONSTANTS, //PID constants for translation
      PathPlannerConstants.ROTATION_PID_CONSTANTS, //PID constants for rotation
      PathPlannerConstants.MAX_TRANSLATION_SPEED, //max possible translation speed of module, not robot itself in m/s
      PathPlannerConstants.ROBOT_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to center of furthest module
      new ReplanningConfig()),
    () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field(go from
      // blue to red)
      // THE ORIGIN WILL REMAIN THE BLUE SIDE

      //Checks if the alliance is red or blue, flips path if.... colour
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return (alliance.get() == DriverStation.Alliance.Red);
      } else {
        return false;
      }
    },
    this);
  }

/**
 * This is useful for odometry paths created through getting position from
  other sensors, such as camera; Basically give bot coordinates, and it auto generates and executes a path to get there.
 * @param targetPos a Pose 2d that represents the target Position of the robot in relative to last odometry reset
 * @return Command: A command that takes the robot to the target Position
 */
public Command pathfindToPose(Pose2d targetPos) {
  PathConstraints constraints = new PathConstraints(PathPlannerConstants.MAX_TRANSLATION_SPEED, PathPlannerConstants.MAX_TRANSLATION_ACCELERATION, PathPlannerConstants.MAX_ROTATION_SPEED, PathPlannerConstants.MAX_ROTATIONAL_ACCELERATION);

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  Command command = AutoBuilder.pathfindToPose(
      targetPos,
      constraints,
      0.0, // Goal end velocity in meters/sec
      0.0 // Rotation delay distance in meters. This is how far the robot should travel
          // before attempting to rotate.
  );

  return command;
}


/**
 * Gets the command of pre-made Paths
 * @param pathName: String that is the path file name
 * @return Command: command that follows the path
 */
public Command getPathfindingCommand(String pathName){
  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
  return AutoBuilder.followPath(path);
}






//Pose just means the robot's coordinates on the field (x, y) as well as it's rotation
/**
 * Resets the pose of the robot. This is done at the start of auto because path planner uses the starting pose in the UI and resets it
 * @param pose the new robot pose
 */
public void resetPose(Pose2d pose) {
  odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
}

/**
 * @return velocities: Returns the velocities with the object ChassisSpeeds; all
 *         velocities are in meters/second
 */
public ChassisSpeeds getVelocities() {
  return ChassisConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
}


  //Field oriented vs robot oriented
  //in robot oriented, pushing forward on the joystick moves the robot forward, based on what direction the robot is facing
  //in field oriented, pushing forward on the joystick moves the robot, forward, based on what direction is forward to the drivers

  public void toggleFieldOriented() {
      isFieldOriented = (isFieldOriented)? false : true;
    }

  /**
   * Enables robot oriented and disables field oriented
   */
  public void enableRobotOriented() {
    isFieldOriented = false;
  }

  /**
   * Enables field oriented and disables robot oriented
   */
  public void enableFieldOriented() {
    isFieldOriented = true;
  }


  public boolean getIsFieldOriented() {
    return isFieldOriented;
  }



  public void zeroHeading() {
    gyro.reset();
  }


  //brake vs coast mode
  //coast: when the motors stop receiving power, they will spin freely until friction stops them
  // brake: when the motors stop receiving power, they will actively attempt to stop their movement and stop 

  /**
   * Sets all drive motors to coast
   */
  public void setDrivesCoast() {
    frontLeft.setDriveCoast();
    frontRight.setDriveCoast();
    backLeft.setDriveCoast();
    backRight.setDriveCoast();
  }

  /**
   * Sets all drive motors to Brake
   */
  public void setDrivesBrake() {
    frontLeft.setDriveBrake();
    frontRight.setDriveBrake();
    backLeft.setDriveBrake();
    backRight.setDriveBrake();
  }


  
  //TODO: test what gyro reads on robot start
  /** 
   * Gyro's value is continuous, it can go past 360
  This function clamps it between 180 and -180 degrees to make it easier to work with.
  Positive values are CCW
  Negative values are CW
   * @return double: the clamped gyro value
   */
  public double getHeadingDegrees() {
    double degrees = Math.IEEEremainder(-gyro.getAngle(), 360);
    if (degrees > 180) {
      return degrees - 360;
    } 
    else if (degrees < -180) {
      return degrees + 360;
    }
    else {
      return degrees;
    }
  }


  public void resetModules() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void straightenModules() {
    frontLeft.setModuleZero();
    frontRight.setModuleZero();
    backLeft.setModuleZero();
    backRight.setModuleZero();
  }


  

  //Rotation 2d is a type of object that stores a rotational value
  /** 
   * Returns a rotation 2d object clamped between 180 and -180 degrees.
   * @return Rotation2d
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }


  
  /** 
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModulePosition.html
   * @return SwerveModulePosition[]
   */
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };
  }

  /**
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModuleState.html
   * @return an array of all the swerve module states
   */
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }


  
  /** 
   * @return Pose2d
   */
  public Pose2d getPose() {
    return odometer.getEstimatedPosition();
  }

  


  
  /** 
   * @return double
   */
  public double getRollRad() {
    return gyro.getRoll() * (Math.PI / 180);
  
  
  }

  
  /** 
   * @return double
   */
  public double getYawRad() {
    return gyro.getYaw() * (Math.PI / 180);
  }

  
  /** 
   * @return double
   */
  public double getPitchRad() {
    return gyro.getPitch() * (Math.PI / 180);
  }




  //TODO: add scaling STDEV based on avg distance from tags as larger distance results in less acurate readings
  //TODO: If needed, parse out readings that deviate x meters from current estimated pose as they are mostly likely the result of erratic system noise
  public void updateRobotOrientationLL() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight-tags", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tags");
    if(Math.abs(gyro.getRate()) > 720) {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    //This checks if the difference between the estimated pose and the new pose is too large.
    //If it is, this is typically due to a sensor error, and the update is discarded
    Pose2d poseChange = mt2.pose.relativeTo(odometer.getEstimatedPosition());
    if (Math.sqrt(Math.pow(poseChange.getX(), 2) + Math.pow(poseChange.getY(), 2)) > 2) {
      doRejectUpdate = true;
    }

    //TODO: Make sure this works as intended
    if (Math.abs(poseChange.getRotation().getDegrees()) > 180) {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate) {
      odometer.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      odometer.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
  }


  /**
   * Uses a trigger input to slow the robot drive speed by up to 50%
   * @param percentageSlow the percentage you wish to slow the robot by
   */
  public void slowDrive(double percentageSlow) {
    this.maxTeleopDriveSpeed = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond - ((ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond) * (0.7 * percentageSlow));
  }

  

  //TODO: add this method to all the swerve commands which enable slow mode
  /**
   * Sets the robot's max speed back to the default value
   */
  public void resetDriveSpeed() {
    this.maxTeleopDriveSpeed = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
  }

  /**
   * Takes in 3 joystick axes to control the swerve chassis + a button to toggle field-oriented
   * @param xSpeed - y-axis of left joystick
   * @param ySpeed - x-axis of left joystick
   * @param turningSpeed - x-axis of right joystick
   * @param isFieldOriented whether or not the robot is field oriented
   */
  public void driveSwerve(double xSpeed, double ySpeed, double turningSpeed) {


    //This is what actually limits the speed by our specified max
    //The limiters ensure that our velocity can only increase by a max amount per second
    //They limit our acceleration to provide a smoother drive
    xSpeed = xLimiter.calculate(xSpeed) * maxTeleopDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxTeleopDriveSpeed;
    turningSpeed = turnLimiter.calculate(turningSpeed) * maxTeleopAngularSpeed;
  
    ChassisSpeeds chassisSpeeds;

    if (isFieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, this.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }


    SwerveModuleState[] moduleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //This resets the speed ratios when a velocity goes to high above the specified max
    //TOOD: Switch to physical max speed instead of set max if robot is too slow
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond);
    this.setModuleStates(moduleStates);
  }




  public void driveSwerveWithPhysicalMax(double xSpeed, double ySpeed, double turningSpeed) {


    //This is what actually limits the speed by our specified max
    //The limiters ensure that our velocity can only increase by a max amount per second
    //They limit our acceleration to provide a smoother drive
    xSpeed = xLimiter.calculate(xSpeed) * ChassisConstants.kPhysicalMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * ChassisConstants.kPhysicalMaxSpeedMetersPerSecond;
    turningSpeed = turnLimiter.calculate(turningSpeed) * ChassisConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;


  
    ChassisSpeeds chassisSpeeds;

    if (isFieldOriented) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, this.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    SwerveModuleState[] moduleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //This resets the speed ratios when a velocity goes too high above the specified max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ChassisConstants.kPhysicalMaxSpeedMetersPerSecond);
    this.setModuleStates(moduleStates);
  }




  /**
   * An alternative driveSwerve method that just takes in a chassisSpeeds object.
   * Used for PathPlanner autonomous
   * @param cSpeeds The desired chassis speeds
   */
  public void driveSwerve(ChassisSpeeds cSpeeds) {

    ChassisSpeeds chassisSpeeds = cSpeeds;
    if (chassisSpeeds.omegaRadiansPerSecond > ChassisConstants.kPhysicalMaxAngularSpeedRadiansPerSecond) {
      chassisSpeeds.omegaRadiansPerSecond = ChassisConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;
    }


    SwerveModuleState[] moduleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //This resets the speed ratios when a velocity goes too high above the specified max
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ChassisConstants.kPhysicalMaxSpeedMetersPerSecond);
    this.setModuleStates(moduleStates); 
  }


  
  /** 
   * Used by drive methods to pass the states created by the swerve kinematics on to the individual modules
   * @param desiredStates An array of the desired states of each module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * This method sets the wheels in an x configuration. All the wheels point outwards from the robot and the wheels diagonal from 
   * eachother have the same angle.The purpose of this position is that in this configuration, the robot becomes very hard to 
   * push which is a good defensive tool.
   */
  public void setModuleXPosition() {
    frontLeft.setDesiredStateNoDeadband(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
    frontRight.setDesiredStateNoDeadband(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
    backLeft.setDesiredStateNoDeadband(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
    backRight.setDesiredStateNoDeadband(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));

  }

  /**
   * Mostly used at the end of commands to stop all motors.
   * This doesn't reset the angle of the turn motor to 0, it just sets the speed of both motors to 0, leaving it in the angle where it was stopped
   */
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }


  /**
   * Updates the LED colors based on the current drive mode
   */
  public void setLEDDriveColors() {
    if(isFieldOriented) {
      LED.enableFieldOrientedEnabledLED();
      LED.disableRobotOrientedEnabledLED();
    }
    else {
      LED.enableRobotOrientedEnabledLED();
      LED.disableFieldOrientedEnabledLED();
    }
  }


  public SwerveDrivePoseEstimator getOdometer() {
    return odometer;
  }


  public SwerveModule getFrontLeftModule() {
    return frontLeft;
  }



  public void sendSmartDashboard() {
    SmartDashboard.putString("Odometry pose", odometer.getEstimatedPosition().toString());


    SmartDashboard.putNumber("Robot Heading", getHeadingDegrees());

    //FRONT RIGHT
    //SmartDashboard.putNumber("drive enc pos FR (m)", frontRight.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos FR (rad)", frontRight.getTurningPosition());
    //SmartDashboard.putNumber("drive enc FR velocity", frontRight.getDriveVelocity());
    //SmartDashboard.putNumber("angular velocity FR", frontRight.getTurnVelocity());
    SmartDashboard.putNumber("turn motor voltage FR", frontRight.getTurnMotorVoltage());

    //FRONT LEFT
    //SmartDashboard.putNumber("drive enc pos FL (m)", frontLeft.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos FL (rad)", frontLeft.getTurningPosition());
    //SmartDashboard.putNumber("drive enc FL velocity", frontLeft.getDriveVelocity());
    //SmartDashboard.putNumber("angular velocity FL", frontLeft.getTurnVelocity()); 
    SmartDashboard.putNumber("turn motor voltage FL", frontLeft.getTurnMotorVoltage());
    
    //BACK RIGHT
    //SmartDashboard.putNumber("drive enc pos BR (m)", backRight.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos BR (rad)", backRight.getTurningPosition());
    //SmartDashboard.putNumber("drive enc BR velocity", backRight.getDriveVelocity());
    //SmartDashboard.putNumber("angular velocity BR", backRight.getTurnVelocity()); 
    SmartDashboard.putNumber("turn motor voltage BR", backRight.getTurnMotorVoltage());

    //BACK LEFT
    //SmartDashboard.putNumber("drive enc pos BL (m)", backLeft.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos BL (rad)", backLeft.getTurningPosition());
    //SmartDashboard.putNumber("drive enc BL velocity", backLeft.getDriveVelocity());
    //SmartDashboard.putNumber("angular velocity BL", backLeft.getTurnVelocity()); 
    SmartDashboard.putNumber("turn motor voltage BL", backLeft.getTurnMotorVoltage());


    //Driver info


    //This method is natively in radians. This turns it into degrees for drivers
    SmartDashboard.putNumber("Angular Velocity (FR)", frontRight.getTurnVelocity() * (180 / Math.PI));

    //Gyro info
    //SmartDashboard.putNumber("Pitch(degrees)", gyro.getPitch());
    //SmartDashboard.putNumber("Roll(degrees)", gyro.getRoll());
    SmartDashboard.putNumber("Yaw(degrees)", gyro.getYaw());
    //SmartDashboard.putNumber("Pitch(radians)", getPitchRad());
    //SmartDashboard.putNumber("Roll(radians)", getRollRad());
    SmartDashboard.putNumber("Yaw(radians)", getYawRad());


  }

  @Override
  public void periodic() {
    setLEDDriveColors();
    updateRobotOrientationLL();
    // This method will be called once per scheduler run
    odometer.updateWithTime(Units.millisecondsToSeconds(System.currentTimeMillis()), getRotation2d(), getSwerveModulePositions());
    sendSmartDashboard();
  }
}