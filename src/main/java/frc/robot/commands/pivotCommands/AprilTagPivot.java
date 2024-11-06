package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AprilTagPivot extends NewAutoPivotToAngle{
    Limelight tagLL;
    Pose2d blueSpeaker = new Pose2d(new Translation2d(0.89, 5.54), new Rotation2d(0));
    Pose2d redSpeaker = new Pose2d(new Translation2d(15.5946, 5.54), new Rotation2d(Math.PI));
    Pose2d mainSpeaker;
    double setpointDegrees;
    SwerveDrivePoseEstimator odometry;
    double bumperToBumper;
    public AprilTagPivot(PivotSubsystem pivot, SwerveDrivePoseEstimator odometry, Limelight tagLL) {
        //Note: Tolerance doesn't actually matter here as we dont end this command upon reaching the tolerance threshold
        super(pivot.getEncoders(), pivot, PivotConstants.PIDConstants.kPivotToleranceThreshold);
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
    public void initialize() {
        System.out.println("fixed it poGgers");
    }

    
    @Override
    public void execute() {

        //TODO: adjust to be front of bumper
        //double distanceMeters = Units.metersToFeet(odometry.getEstimatedPosition().getTranslation().getDistance(mainSpeaker.getTranslation())) -(20/12);
        //double setpointDegrees = (9.5/5)*distanceMeters - 83;
        //SmartDashboard.putNumber("test/distance in feet", distanceMeters);
        //SmartDashboard.putNumber("test/set degrees", setpointDegrees);


        //This distance is just using camera to tag instead of botpose
        //This current returns the distance from the april tag to the camera. needs to be adjusted to robot to front of bumper
        boolean tagDetected = tagLL.targetDetected();
        
        if (tagDetected) {
            double[] altDistance = tagLL.getTagPoseRobotSpace();
            double newDistance = Math.sqrt(Math.pow(altDistance[0], 2) + Math.pow(altDistance[2], 2));
            bumperToBumper = newDistance - 1.2605;
            SmartDashboard.putNumber("test/WOo", Units.metersToFeet(bumperToBumper));
            double realSetAngle = (9.5/5)*Units.metersToFeet(bumperToBumper) - 83;
            SmartDashboard.putNumber("test/desired angle", realSetAngle);
            setpointDegrees = realSetAngle;
            SmartDashboard.putNumber("test/dActualAngle", pivot.getEncoders());
            SmartDashboard.putNumber("Angle to apriltag", tagLL.getTX());

        }


        
        if (bumperToBumper> 10) {
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
