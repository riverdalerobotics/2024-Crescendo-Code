// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;

/** Add your docs here. */
public class OI {
    XboxController moveController = new XboxController(OperatorConstants.kDriverControllerPort);
    XboxController intakeController = new XboxController(OperatorConstants.kOperatorControllerPort);

/** 
 * Driver controls ||
 * returns up/down axis of left joystick
 * @return double
 */
public double xSpeed(){
    System.out.println(-moveController.getLeftY());
    return -moveController.getLeftY();
}


/** 
 * Driver controls ||
 * returns left/right axis of left joystick
 * @return double
 */
//y is left right
public double ySpeed(){
    return -moveController.getLeftX();
}



/** 
 * Driver controls ||
 * returns left/right axis of right joystick 
 * @return double
 */
public double rotate(){
    return moveController.getRightX();
}

public boolean testButton() {
    return moveController.getBButton();
}

//TODO: figure out which button this is
/** 
 * Driver controls ||
 * Used to set the drive mode to robot oriented
 * @return true when the movement controller's right stick is pressed
 */
public boolean engageRobotOriented(){
    return moveController.getRightStickButtonPressed();
}

/**
 * Driver controls ||
 * Used to set the drive mode to field oriented
 * @return true when the movement controller's left stick is pressed
 */
public boolean engageFieldOriented() {
    return moveController.getLeftStickButtonPressed();
}


public boolean engageDriveBrakeMode() {
    return moveController.getYButton();
}

/** 
 * Driver controls ||
 * Used to reset robot gyro
 * returns true when the movement controller's start button is pressed
 * @return boolean
 */
public boolean resetGyro() {
    return moveController.getStartButtonPressed();
}

/**
 * Driver controls ||
 * Puts the robot in robot oriented and auto drives to note in both axis
 * @return true when the driver's left trigger button is pressed basically all the way down
 */
public boolean engageNoteAlignAssist() {
    if (moveController.getLeftTriggerAxis() >= 0.330){ //beach bots lol
        return true;
    } else{
        return false;
    }
}

/**
 * Driver controls ||
 * Slows the robot down by 50% when held
 * @return 0.5 if bumper is down
 */
public double engageSlowMode() {
    if(moveController.getLeftBumper()){
        return 1;
    } else{
        return 0;
    }
}


/**
 * Driver controls ||
 * Engages Robot to auto move to closest predefined shooting position on the field
 * @return True if drivfer left axis is down, false if axis is not down
 */
public boolean engageAutoMoveToPredefined() {
    if(moveController.getLeftTriggerAxis() >= 0.330){
        return true;
    } else{
        return false;
    }
}

// public boolean spinIntake(){
//     return intakeController.getLeftTriggerAxis() >= 0.330;
// }


/**
 * Driver controls
 * <p>
 * Used to pivot and shoot at high intake once wheels are revved
 * @return true as long as the right trigger is down 
 */
public boolean rotateRobot() {
    if (moveController.getRightTriggerAxis()>0.1){
        return true;
    } else{
        return false;
    }
}


/**
 * Driver controls ||
 * When the robot is ready to shoot, the driver pressed the right bumper to power the indexer to fire the note.
 * This can be used at any time during manual operation, but during the auto shoot command, it can only be activated once the position is ready
 * @return true as long as the move controller's right bumper is held down
 */
public boolean shoot() {
    return moveController.getRightBumper();
}


public boolean engageXModulePosition() {
    return moveController.getXButton();
}
/**
 * Controller imput to shoot from 5 ft
 * TODO: find a better button for this... kinda change around intake controlls...
 * @return intake controller back button
 */
// public boolean pivotFromfar(){
//     return intakeController.getRightStickButton();
// }


/**
 * Driver Controls 
 * <p>
 * Used to bring the arm up or down
 * @return The angle of the Dpad button being pressed on the driver controller
 */
public int powerArm(){
    return moveController.getPOV();
}


/**
 * Driver Controls 
 * <p>
 * Used to enable moving swerve using hand signals
 * @return true when b button pressed
 */
public boolean handSignalMove(){
    return moveController.getBButton();
}






/**
 * Operator controls 
 * <p>
 * Used to pivot the arm down to the intake position. Meant to be used in conjuction with
 * engageIntake() which checks if the same button is held and powers intake
 * @return true when the operator's left bumper is pressed
 */
public boolean pivotToIntakePosition() {
    return intakeController.getLeftBumperPressed();
}

/**
 * Operator control 
 * <p>
 * Used to engage the fly wheels and power them to intake speed. Meant to be used in conjuction with
 * pivotToIntakePosition() which checks if the same button is pressed and pivots to intake angle
 * @return true as long as the operator's left bumper is pressed
 */
public boolean engageIntake() {
    return intakeController.getLeftBumper();
}




/** 
 * Operator controls ||
 * Uses to automatically pivot the arm up to the speaker shoot angle
 * @return true when the operator controller's right bumper is pressed
 */
public boolean pivotToSubwooferShoot(){
    return intakeController.getXButtonPressed();
}








/**
 * Operator controls ||
 * Used to engage the auto PID spinup for shooting fly wheels.
 * @return true when the right bumper is down
 */
public boolean engageAutoShootSpinup(){
    /*if (intakeController.getRightTriggerAxis() > 0.2) {
        System.out.println("POG RUN SHOOT");
    }
    else {
        System.out.println("NO");
    }*/
    return intakeController.getRightBumper();
    
}

/**
 * Operator controls ||
 * Used to enable manual operator of the pivot
 * Returns true when the left stick is pressed down
 * @return
 */
public boolean enableManualRotation() {
    return intakeController.getLeftStickButtonPressed();
}
//I think this would work for testing
// public boolean enableManualIntakeControl(){
//     return intakeController.getRightStickButton();
// }


/**
 * Operator controls ||
 * Used to manually pivot the arm when manual pivot is enabled.
 * @return the value of the left joystick's y axis
 */
public double pivotArm() {
    return intakeController.getLeftY();
}



public boolean testX() {
    return intakeController.getXButton();
}

/**
 * Operator controls ||
 * Used to engage the fly wheels at the feed RPS.
 * @return true as long as the intake controller's A button is held
 */
public boolean shootFeed() {
    if(intakeController.getLeftTriggerAxis()>0.2){
        return true;
    } else{
        return false;
    }
}

/**
 * Operator controls 
 * <p>
 * Used to pivot to shoot opposite from intake side
 * <p>
 * :)
 * @return true when the intake controller's Right Bumper is pressed
 */
public boolean pivotToReverseShot() {
    return intakeController.getRightBumperPressed();
}

/**
 * Operator controls
 * <p>
 * Used to pivot and shoot at low intake once wheels are revved
 * @return true as long as the Left Trigger is pressed
 */
public boolean pivotAndShootLowFeed() {
    if(intakeController.getLeftTriggerAxis()>0.2){
        return true;
    } else{
        return false;
    }
}

/**
 * Operator controls
 * <p>
 * Used to pivot and shoot at high intake once wheels are revved
 * @return true as long as the Right Trigger is held
 */
public boolean revHighFeed() {
    if(intakeController.getRightTriggerAxis()>0.2){
        return true;
    } else {
        return false;
    }
    
}
/**
 * Operator controls
 * <p>
 * Used to pivot and shoot at high intake once wheels are revved
 * @return true as long as the Right Trigger is clicked
 */
public boolean pivotToHighFeed() {
    if(intakeController.getRightTriggerAxis()>0.2){
        return true;
    } else {
        return false;
    }
}

/**
 * Operator controls
 * <p>
 * Used to rev the arm for shooting at amp. The driver is still given final say to fire the shot.
 * Used in conjuction with pivotToAmp() which pivots the arm to anp angle when operator's Y button is pressed
 * @return true as long as the operator's Y button is held
 */
public boolean revAmp() {
    return intakeController.getYButton();
}

/**
 * Operator controls
 * <p>
 * Used to pivot to the amp angle. Used in conjunction with revAmp() which
 * revs the arm for amp shooting as long as Y is held
 * @return true when operator's Y button is pressed
 */
public boolean pivotToAmp() {
    return intakeController.getYButtonPressed();
}

/**
 * Operator controls ||
 * Used the manually power the arm flywheels when manual intake is enabled
 * @return the value of the operator controller's right joystick's y axis
 */
public double manualPowerIntake() {
    return intakeController.getRightY();
}

//CUR UNUSED (2 methods below)

//-------------------------------------------------------------------
/**
 * Operator controls ||
 * Used to manually push arm into hard stop in the event of an encoder issue
 * @return true as long as the operator controller's back button is pressed
 */
public boolean pushArmIntoHardstop() {
    return intakeController.getBackButton();
}


/**
 * Operator controls ||
 * Uses to engage the tuck command in the event the arm's encoder is thrown off. 1 of 2 buttons to do this
 * @return true as long as the operator controller's start button is pressed
 */
public boolean tuckArm2() {
    return intakeController.getStartButton();
}
//------------------------------------------------------------------

/**
 * Operator controls
 * <p>
 * Resets arm to hard stop inside bumpers.
 * @return True when the operator controller's start button is pressed
 */
// public boolean resetArmMinPos() {
//     return intakeController.getStartButtonPressed();
// }

/**
 * Operator controls
 * <p>
 * Resets arm to intake hard stop
 * @return True when the operator controller's back button is pressed
 */
public boolean resetArmMaxPos() {
    return intakeController.getRightStickButton();
}
}
