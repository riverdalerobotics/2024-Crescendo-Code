package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

public class Limelight {
      //Adapted from https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
    //all measurements in degrees

    /**The NetworkTable we are reading limelight values from*/
    NetworkTable table;

    double[] botPose;  

    
    /** Creates a new limelight object. Sets up the NetworkTable object */
    /** For the limelightName, ll3: "limelight-note", ll2: "limelight-tags" */
    public Limelight(String limelightName ){
        table = NetworkTableInstance.getDefault().getTable(limelightName);
        botPose = getBotPose();
    }

    
    public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);
    }
 
    /** 
     * @return The horizontal offset in degrees. Returns 360 if no target found.
     */
    public double getTX(){
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(360);
    }


    /**
     * @return The vertical offset in degrees. Returns 360 if no target found
     */
    public double getTY(){
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(360);
    }


    /**
     * @return true or false if theres a target
     */
    public boolean targetDetected(){
        NetworkTableEntry tv = table.getEntry("tv"); 
        return tv.getBoolean(false);
    }


    /**
     * @return the latency
     */
    // public double getTL(){
    //     NetworkTableEntry tl = table.getEntry("tl");
    //     return tl.getDouble(-1);
    // }


    /**
     * @return the target area (of the camera stream)
     */
    public double getTA(){
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0);
    }


    /**
     * @return The distance to the target
     */
    public double calculateDistanceNote(){
        return Math.abs(LimelightConstants.noteHeight-LimelightConstants.noteLimelightMountHeight)/Math.tan((LimelightConstants.noteLimelightMountAngle + getTX())*Math.PI/180);
    }

    
    /**
     * @return The robot pose with relative to the center of the field(X position, y position, z position, Roll,Pitch,Yaw). Returns (0,0,0,0,0,0) if no value
     */
    public double[] getBotPose(){
        NetworkTableEntry botpose = table.getEntry("botpose");
        return botpose.getDoubleArray(new double[6]);
        }


    /**
     * @return The robot yaw
     */
    public double getYaw(){
        botPose=getBotPose();
        return botPose[5];
     }

    /**
     * @return The robot x position pose relative to the center of the field
     */
    public double getXPosition(){
        botPose=getBotPose();
        return botPose[0];
    }

    /**
     * @return The robot y position pose relative to the center of the field
     */
    public double getYPosition(){
        botPose=getBotPose();
        return botPose[1];
    }

    public Pose2d getBotPoseOdometryNotation(){

        double convertedX = this.getXPosition() + Constants.LimelightConstants.ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT[0];
        double convertedY = this.getYPosition() + Constants.LimelightConstants.ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT[1];
        double convertedTheta = this.getYaw() + Constants.LimelightConstants.ORIGIN_PATHPLANNER_FROM_ORIGIN_LIMELIGHT[0];//TODO: This is 0 right now as we think converted is the same.

        return new Pose2d(new Translation2d(convertedX, convertedY), new Rotation2d(convertedTheta));
    }
    


 
}