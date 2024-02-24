// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PathPlannerConstants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  public Chassis() {
    AutoBuilder.configureHolonomic(
        this::getPos,
        this::setPos,
        this::getVelocities,
        this::drive,
        new HolonomicPathFollowerConfig(
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
  other sensors, such as camera
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return pos: The coordinates of the robot in a Pose2d(xCoord, yCoord, zCoord)
   */
  public Pose2d getPos() {
    return null;
  }

  /**
   * Set pos to the new Pose2d
   * 
   * @param newPos: The new pos odometry should "be at"
   */
  public void setPos(Pose2d newPos) {
  }

  /**
   * @return velocities: Returns the velocities with the object ChassisSpeeds; all
   *         velocities are in meters/second
   */
  public ChassisSpeeds getVelocities() {
    return null;
  }

  /**
   * 
   * @param botVelocities velocities of the bot in each axis in meters per second.
   *                      Given through the ChassisSpeeds object
   */
  public void drive(ChassisSpeeds botVelocities) {
  }
}
