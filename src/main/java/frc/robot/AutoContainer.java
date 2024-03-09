package frc.robot;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Limelight;
// import frc.robot.subsystems.SwerveChassisSubsystem;

// /** Add your docs here. */
// public class AutoContainer {
//     SwerveChassisSubsystem chassis;
//     Limelight cameraNote;

//     AutoContainer(SwerveChassisSubsystem sChassis){
//         this.chassis = sChassis;
//         cameraNote = new Limelight("limelight-note");
//     }

//     public Command pathFindNoteUsingLimelight(){
//         Pose2d botCurrentPos = this.chassis.getPose();
//         Pose2d notePose = new Pose2d(
//             botCurrentPos.getX() + cameraNote.getXDisplacementFromNote(),
//             botCurrentPos.getY() + cameraNote.getYDisplacementFromNote(),
//             new Rotation2d(
//                 botCurrentPos.getRotation().getDegrees() + cameraNote.getTX()
//             )
//         );

//         return this.chassis.pathfindToPose(notePose);
//     }

// }
