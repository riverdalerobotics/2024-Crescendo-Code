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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import com.kauailabs.navx.frc.AHRS;


public class SwerveChassisSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(ChassisConstants.kFrontLeftDriveMotorPort, 
  ChassisConstants.kFrontLeftTurningMotorPort);

  private final SwerveModule frontRight = new SwerveModule(ChassisConstants.kFrontRightDriveMotorPort,
  ChassisConstants.kFrontRightTurningMotorPort);

  private final SwerveModule backLeft = new SwerveModule(ChassisConstants.kBackLeftDriveMotorPort,
  ChassisConstants.kBackLeftTurningMotorPort);

  private final SwerveModule backRight = new SwerveModule(ChassisConstants.kBackRightDriveMotorPort,
  ChassisConstants.kBackRightTurningMotorPort);

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
  private double maxTeleopDriveSpeed;
  private double maxTeleopAngularSpeed;
  private boolean slowMode;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(ChassisConstants.kDriveKinematics,
  getRotation2d(), getSwerveModulePositions());


  /** Creates a new SwerveChassisSubsystem. */
  public SwerveChassisSubsystem() {
    this.maxTeleopDriveSpeed = ChassisConstants.kTeleDriveMaxSpeedMetersPerSecond;
    this.maxTeleopAngularSpeed = ChassisConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    this.xLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turnLimiter = new SlewRateLimiter(ChassisConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    resetModules();



    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
    } catch (Exception e) {
    }
  }).start();
    
  }



  public void zeroHeading() {
    gyro.reset();
  }


  //Gyro's value is continuous, it can go past 360
    //This function clamps it between 180 and -180 degrees to make it easier to work with
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


  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }


  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };
  }


  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d newPose) {
    odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), newPose);
  }

  public double getRollRad() {
    return gyro.getRoll() * (Math.PI / 180);
  }

  public double getYawRad() {
    return gyro.getYaw() * (Math.PI / 180);
  }

  public double getPitchRad() {
    return gyro.getPitch() * (Math.PI / 180);
  }


  public void driveSwerve(double xSpeed, double ySpeed, double turningSpeed, boolean isFieldOriented) {

    //I dont think this actually does anything
    xSpeed = xLimiter.calculate(xSpeed) * maxTeleopDriveSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * maxTeleopDriveSpeed;
    turningSpeed = turnLimiter.calculate(turningSpeed) * maxTeleopAngularSpeed;

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("tSpeed", turningSpeed);


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



  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop()
    backLeft.stop();
    backRight.stop();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getSwerveModulePositions());


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
  }
}