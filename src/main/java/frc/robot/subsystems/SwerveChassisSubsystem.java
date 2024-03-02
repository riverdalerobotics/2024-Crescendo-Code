// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PathPlannerConstants;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;


public class SwerveChassisSubsystem extends SubsystemBase {
  
  public boolean commandActive = false;
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
  private boolean slowMode;
  private boolean isFieldOriented;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(ChassisConstants.kDriveKinematics,
  getRotation2d(), getSwerveModulePositions());
  private final OI opInput;


  /** Creates a new SwerveChassisSubsystem. */
  public SwerveChassisSubsystem(OI opInp) {
    this.maxTeleopDriveSpeed = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
    this.maxTeleopAngularSpeed = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    this.xLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond);
    this.yLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationMetersPerSecond);
    this.turnLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAngularAccelerationRadiansPerSecond);
    resetModules();

    opInput = opInp;



    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
    } catch (Exception e) {
    }
  }).start();

  //Pathplanner Stuff
  AutoBuilder.configureHolonomic( //Configures pathfinder with basic constraints and functionality of robot
    this::getPose, //Pose2d, datatype
    this::resetPose, //Pose2d, datatype
    this::getVelocities, 
    this::driveSwerve,
    new HolonomicPathFollowerConfig( //Object with the configurations for our drive train, particularly max speeds, PID Constants, and radius of our base
      PathPlannerConstants.TRANSLATION_PID_CONSTANTS,
      PathPlannerConstants.ROTATION_PID_CONSTANTS,
      PathPlannerConstants.MAX_TRANSLATION_SPEED,
      PathPlannerConstants.ROBOT_BASE_RADIUS,
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

    // REDUNDANT
  // /**
  // * This is useful for odometry paths created through getting position from
  // other sensors, such as camera
  // * @param currentPos: a Pose2d that represents the current position of the
  // robot
  // * @param targetPos: a Pose2d that represents the target position of the robot
  // */
  // public PathPlannerPath createCustomStraightLinePath(Pose2d currentPos, Pose2d
  // targetPos) {
  // // Create a list of bezier points from poses. Each pose represents one
  // waypoint.
  // // The rotation component of the pose should be the direction of travel. Do
  // not
  // // use holonomic rotation.

  // double changeInX = targetPos.getX() - currentPos.getX();
  // double changeInY = targetPos.getY() - currentPos.getY();
  // double changeInAngle = targetPos.getRotation().getDegrees() -
  // currentPos.getRotation().getDegrees();

  // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  // new Pose2d(currentPos.getX()+(changeInX/3), currentPos.getY()+(changeInY/3),
  // Rotation2d.fromDegrees(currentPos.getRotation().getDegrees()+(changeInAngle/3))),
  // new Pose2d(currentPos.getX()+(changeInX/3*2),
  // currentPos.getY()+(changeInY/3*2),
  // Rotation2d.fromDegrees(currentPos.getRotation().getDegrees()+(changeInAngle/3*2))),
  // new Pose2d(targetPos.getX(), targetPos.getY(),
  // Rotation2d.fromDegrees(targetPos.getRotation().getDegrees()))
  // );

  // // Create the path using the bezier points created above
  // PathPlannerPath path = new PathPlannerPath(
  // bezierPoints,
  // new PathConstraints(MAX_TRANSLATION_SPEED, MAX_TRANSLATION_ACCELERATION,
  // MAX_ROTATION_SPEED, MAX_ROTATIONAL_ACCELERATION), // The constraints for this
  // path. If using a
  // // differential drivetrain, the angular constraints
  // // have no effect.
  // new GoalEndState(0.0, 0) // Goal end state. You can set a holonomic rotation
  // here. If
  // // using a differential drivetrain, the rotation will have no
  // // effect.
  // );

  // // Prevent the path from being flipped if the coordinates are already correct
  // path.preventFlipping = true;

  // return path;

  // }


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



//TODO: DELETE
// /**
//  * Set pos to the new Pose2d
//  * 
//  * @param newPos: The new pos odometry should "be at"
//  */
// public void setPos(Pose2d newPos) {
// }


//resets the pose of the robot. Is done at the start of auto because path planner uses the starting pose in the UI and resets it
public void resetPose(Pose2d pose) {
  odometer.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose);
}

/**
 * @return velocities: Returns the velocities with the object ChassisSpeeds; all
 *         velocities are in meters/second
 */
public ChassisSpeeds getVelocities() {
  return null;
}



  public void toggleFieldOriented() {
      isFieldOriented = (isFieldOriented)? false : true;
    }

  public void disableFieldOriented() {
    isFieldOriented = false;
  }

  public void zeroHeading() {
    gyro.reset();
  }


  
  /** 
   * Gyro's value is continuous, it can go past 360
  This function clamps it between 180 and -180 degrees to make it easier to work with
   * @return double: the clamped gyro value
   */
  
  public double getHeadingDegrees() {
    return Math.IEEEremainder(-gyro.getAngle(), 360);
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


  
  /** 
   * @return Rotation2d
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }


  
  /** 
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
   * @return Pose2d
   */
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  
  /** 
   * Resets the swerve odometer's perceived position on the field. This is useful for adjusting odometry based
   * on apriltag distances when odometry becomes inaccurate 
   * @param newPose
   */
  public void resetOdometry(Pose2d newPose) {
    odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), newPose);
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

  /**
   * Takes in 3 joystick axes to control the swerve chassis + a button to toggle field-oriented
   * @param xSpeed - y-axis of left joystick
   * @param ySpeed - x-axis of left joystick
   * @param turningSpeed - x-axis of right joystick
   * @param isFieldOriented
   */
  public void driveSwerve(double xSpeed, double ySpeed, double turningSpeed) {


    //TODO: try getting rid of these
    //I dont think this actually does anything
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




  public void driveSwerve(ChassisSpeeds cSpeeds) {

    ChassisSpeeds chassisSpeeds = cSpeeds;


    SwerveModuleState[] moduleStates = ChassisConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //This resets the speed ratios when a velocity goes to high above the specified max
    //TOOD: Switch to physical max speed instead of set max if robot is too slow
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond);
    this.setModuleStates(moduleStates);
  }


  
  /** 
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getSwerveModulePositions());


    SmartDashboard.putNumber("XSpeed", opInput.xSpeed() * maxTeleopDriveSpeed);
    SmartDashboard.putNumber("YSpeed", opInput.ySpeed() * maxTeleopDriveSpeed);
    SmartDashboard.putNumber("TurnSpeed", opInput.rotate() * maxTeleopAngularSpeed);

    SmartDashboard.putNumber("Robot Heading", getHeadingDegrees());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());


    SmartDashboard.putNumber("drive enc pos FR (m)", frontRight.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos FR (rad)", frontRight.getTurningPosition());
    SmartDashboard.putNumber("drive enc FR velocity", frontRight.getDriveVelocity());
    SmartDashboard.putNumber("angular velocity FR", frontRight.getTurnVelocity());

    SmartDashboard.putNumber("drive enc pos FL (m)", frontLeft.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos FL (rad)", frontLeft.getTurningPosition());
    SmartDashboard.putNumber("drive enc FL velocity", frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("angular velocity FL", frontLeft.getTurnVelocity());  
    
    SmartDashboard.putNumber("drive enc pos BR (m)", backRight.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos BR (rad)", backRight.getTurningPosition());
    SmartDashboard.putNumber("drive enc BR velocity", backRight.getDriveVelocity());
    SmartDashboard.putNumber("angular velocity BR", backRight.getTurnVelocity()); 

    SmartDashboard.putNumber("drive enc pos BL (m)", backLeft.getDrivePosition());
    SmartDashboard.putNumber("turn enc pos BL (rad)", backLeft.getTurningPosition());
    SmartDashboard.putNumber("drive enc BL velocity", backLeft.getDriveVelocity());
    SmartDashboard.putNumber("angular velocity BL", backLeft.getTurnVelocity()); 


    //Driver info

    //Changes method to degrees for drivers
    SmartDashboard.putNumber("Absolute Turn Angle (FR)", frontRight.getTurningPosition() * (180 / Math.PI));
    SmartDashboard.putNumber("Velocity (FR)", frontRight.getDriveVelocity());

    //This method is natively in radians. This turns it into degrees for drivers
    SmartDashboard.putNumber("Angular Velocity (FR)", frontRight.getTurnVelocity() * (180 / Math.PI));

    //Gyro info
    SmartDashboard.putNumber("Pitch(degrees)", gyro.getPitch());
    SmartDashboard.putNumber("Roll(degrees)", gyro.getRoll());
    SmartDashboard.putNumber("Yaw(degrees)", gyro.getYaw());
    SmartDashboard.putNumber("Pitch(radians)", getPitchRad());
    SmartDashboard.putNumber("Roll(radians)", getRollRad());
    SmartDashboard.putNumber("Yaw(radians)", getYawRad());
    SmartDashboard.putBoolean("CommandActive", commandActive);
  }
}