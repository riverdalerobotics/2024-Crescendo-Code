// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
    return moveController.getLeftY();
}


/** 
 * Driver controls ||
 * returns left/right axis of left joystick
 * @return double
 */
//y is left right
public double ySpeed(){
    return moveController.getLeftX();
}



/** 
 * Driver controls ||
 * returns left/right axis of right joystick 
 * @return double
 */
public double rotate(){
    return moveController.getRightX();
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
 * Puts the robot in robot oriented so driver pushes y axis forward, while note aim assist is happening 
 * @return true when the driver's left trigger button is pressed basically all the way down
 */
public boolean engageNoteAlignAssist() {
    if (moveController.getRightTriggerAxis() >= 0.330){ //beach bots lol
        return true;
    } else{
        return false;
    }
}

/**
 * Driver controls ||
 * Slows the robot down by up to 50% based on how much the trigger is pressed
 * @return the value of the movement controller's left trigger
 */
public double engageSlowMode() {
    return moveController.getLeftTriggerAxis();
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



/**
 * Operator controls ||
 * Used to pivot the arm down to the intake position.
 * @return true when the operator's X button is pressed
 */
public boolean pivotToIntakePosition() {
    return intakeController.getLeftBumperPressed();
}




/** 
 * Operator controls ||
 * Uses to automatically pivot the arm up to the speaker shoot angle
 * @return true when the operator controller's right bumper is pressed
 */
public boolean shootPos(){
    return intakeController.getRightBumperPressed();
}






/**
 * Operator controls ||
 * Used to engage the auto PID spinup of the intake fly wheels.
 * @return true when the left trigger axis of the operator controller is pushed in above 0.2
 */
public boolean engageAutoIntakeSpinup(){
    return intakeController.getLeftTriggerAxis() > 0.2;
}

/**
 * Operator controls ||
 * Used to engage the auto PID spinup for shooting fly wheels.
 * @return true when the right trigger axis of the operator controller is pushed in above 0.2
 */
public boolean engageAutoShootSpinup(){
    return intakeController.getRightTriggerAxis() > 0.2;
}

/**
 * Operator controls ||
 * Used to enable manual operator of the pivot
 * Returns true when the right stick is pressed down
 * @return
 */
public boolean enableManualRotation() {
    return intakeController.getLeftStickButtonPressed();
}
//I think this would work for testing
public boolean enableManualIntakeControl(){
    return intakeController.getRightStickButton();
}


/**
 * Operator controls ||
 * Used to manually pivot the arm when manual pivot is enabled.
 * @return the value of the left joystick's y axis
 */
public double pivotArm() {
    return intakeController.getLeftY();
}



/**
 * Operator controls ||
 * Used the manually power the arm flywheels when manual intake is enabled
 * @return the value of the operator controller's right joystick's y axis
 */
public double manualPowerIntake() {
    return intakeController.getRightY();
}

/**
 * Operator controls ||
 * Uses to engage the tuck command in the event the arm's encoder is thrown off. 1 of 2 buttons to do this
 * @return true as long as the operator controller's back button is pressed
 */
public boolean tuckArm1() {
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


}
