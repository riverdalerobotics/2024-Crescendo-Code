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
 * Used to toggle the robot driving between robot and field oriented
 * returns true when one of the movement controller's center buttons is pressed
 * @return boolean
 */
public boolean toggleFieldOriented(){
    return moveController.getBackButtonPressed();
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
 * Returns true when the driver's left trigger button is pressed basically all the way down
 * @return
 */
public boolean engageNoteAlignAssist() {
    if (moveController.getLeftTriggerAxis() >= 0.330){ //beach bots lol
        return true;
    } else{
        return false;
    }
}








/** 
 * Operator controls ||
 * Used to engage the intake mechanisms (power shooter & indexing belt inwards)
 * Returns true while the operator's X button is held down
 * @return boolean
 */
public boolean powerIntakeMechanisms() {
    return intakeController.getXButton();
}

/**
 * Operator controls ||
 * Used to pivot the arm down to the intake position. Used in tandem with the method above to pivot and start spinning intake
 * Returns true when the operator's X button is pressed
 * @return
 */
public boolean pivotToIntakePosition() {
    return intakeController.getXButtonPressed();
}


/** 
 * Hold left bumper to activate automatic pickup
 * @return boolean
 */
//other things
//Go to pickup position
public boolean pickUpPos(){
    return intakeController.getLeftBumper();
}


/** 
 * @return boolean
 */
//Go to shoot position
public boolean shootPos(){
    return intakeController.getAButton();
}

/** 
 * @return double
 */
public double manualBeltSpeed(){
    return intakeController.getRightY();
}


/** 
 * Operator controls ||
 * used to manually power the shooter motors
 * Returns the right trigger axis of the operator controller
 * @return double
 */
//Shoot
//TODO: make this a boton!! it is now in "testing mode"
public double manualShoot(){
    return intakeController.getLeftY();
}

/** 
 * @return boolean
 */
public boolean shoot(){
    return intakeController.getAButtonPressed();
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
 * @return boolean
 */
//Go to drivePos
public boolean drivePos(){
    return intakeController.getLeftStickButton();
}



/** 
 * @return boolean
 */
//OTHER THINGS
// STOP arm
public boolean stopArm(){
    return intakeController.getStartButton();
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
public boolean disableManualIntakeControl(){
    return intakeController.getRightStickButton();
}


/**
 * Operator controls ||
 * Used to manually pivot the arm when manual pivot is enabled.
 * Returns the value of the left joystick's y axis
 * @return
 */
public double pivotArm() {
    return intakeController.getLeftY();
}


}
