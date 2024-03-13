// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.SwerveChassisSubsystem;

/** Add your docs here. */
public class AutoBuilderTools {
    SwerveChassisSubsystem chassis;
    Limelight intakeCamera;
    Limelight shootCamera;
    // Pose2d lastPose;
    PathPlannerPath lastPath;

    AutoBuilderTools(Limelight intakeCamera, Limelight shootCamera) {
        this.intakeCamera = intakeCamera;
        this.shootCamera = shootCamera;
    }

    // public void setLastPose(Pose2d pos){
    //     lastPose = pos;
    // }

    // public void setLastPath(PathPlannerPath path){
    //     lastPath = path;
    // }

    // public Pose2d getEndPoseOfPath(PathPlannerPath path){
    //     List<Pose2d> arrayOfCoords = path.getPathPoses();
    //     Pose2d pos = arrayOfCoords.get(arrayOfCoords.size()-1);
    //     return pos;
    // }

    // public Command pathFindNoteUsingLimelight() {
    //     Pose2d botCurrentPos = this.chassis.getPose();
    //     Pose2d notePose = new Pose2d(
    //             botCurrentPos.getX() + intakeCamera.getXDisplacementFromNote(),
    //             botCurrentPos.getY() + intakeCamera.getYDisplacementFromNote(),
    //             new Rotation2d(
    //                     botCurrentPos.getRotation().getDegrees() + intakeCamera.getTX()));
        
    //     // setLastPose(getEndPoseOfPath(lastPath));
    //     return this.chassis.pathfindToPose(notePose);
    // }

    public void resetOdometryUsingCamera(){
        chassis.resetPose(shootCamera.getBotPoseOdometryNotation());
    }
}
