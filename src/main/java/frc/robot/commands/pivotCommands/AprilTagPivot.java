package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AprilTagPivot extends NewAutoPivotToAngle{
    Limelight tagLL;
    Pose2d blueSpeaker = new Pose2d(new Translation2d(0.89, 5.54), new Rotation2d(0));
    Pose2d redSpeaker = new Pose2d(new Translation2d(0, 5.54), new Rotation2d(Math.PI));
    Pose2d mainSpeaker;
    SwerveDrivePoseEstimator odometry;
    public AprilTagPivot(PivotSubsystem pivot, SwerveDrivePoseEstimator odometry, Limelight tagLL) {
        //Note: Tolerance doesn't actually matter here as we dont end this command upon reaching the tolerance threshold
        super(0, pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold);
        this.odometry = odometry;
        this.tagLL = tagLL;

        if (OperatorConstants.allianceColor == Alliance.Red) {
            mainSpeaker = redSpeaker;
        }
        else {
            mainSpeaker = blueSpeaker;
        }
        
    }

    
    @Override
    public void execute() {
        //TODO: adjust to be front of bumper
        double distanceMeters = odometry.getEstimatedPosition().getTranslation().getDistance(mainSpeaker.getTranslation());
        double setpointDegrees = (9.5/5)*distanceMeters - 83;


        //This distance is just using camera to tag instead of botpose
        //This current returns the distance from the april tag to the camera. needs to be adjusted to robot to front of bumper
        boolean tagDetected = tagLL.targetDetected();
        if (tagDetected) {
            double[] altDistance = tagLL.getTagPoseRobotSpace();
            double newDistance = Math.sqrt(Math.pow(altDistance[0], 2) + Math.pow(altDistance[1], 2));
        }

        
        if (distanceMeters > 5) {
            //This should check if the distance is beyond the feasible limit
        }
        else {
        pivot.setPivotAngleDegrees(setpointDegrees);
        executeMethod();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
