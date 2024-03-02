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
 * Movement controls ||
 * returns up/down axis of left joystick
 * @return double
 */
public double xSpeed(){
    return moveController.getLeftY();
}


/** 
 * Movement controls ||
 * returns left/right axis of left joystick
 * @return double
 */
//y is left right
public double ySpeed(){
    return moveController.getLeftX();
}


/** 
 * Movement controls ||
 * returns left/right axis of right joystick 
 * @return double
 */
public double rotate(){
    return moveController.getRightX();
}


//TODO: figure out which button this is
/** 
 * Movement controls ||
 * Used to toggle the robot driving between robot and field oriented
 * returns true when one of the movement controller's center buttons is pressed
 * @return boolean
 */
public boolean toggleFieldOriented(){
    return moveController.getBackButtonPressed();
}


/** 
 * Movement controls ||
 * Used to reset robot gyro
 * returns true when the movement controller's start button is pressed
 * @return boolean
 */
public boolean resetGyro() {
    return moveController.getStartButtonPressed();
}








/** 
 * Operator controls ||
 * Used to engage the intake mechanisms (power shooter & indexing belt inwards)
 * returns true when the operator controller's x button is pressed
 * @return boolean
 */
public boolean powerIntakeMechanisms() {
    return intakeController.getXButton();
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
    return intakeController.getRightBumper();
}

/** 
 * @return double
 */
public double beltSpeed(){
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
public double manShoot(){
    return intakeController.getRightTriggerAxis();
}

/** 
 * @return boolean
 */
public boolean shoot(){
    return intakeController.getAButtonPressed();
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
// Arm Up and down
public boolean moveArmUp(){
    return intakeController.getRightBumperPressed();
}

/** 
 * @return boolean
 */
public boolean moveArmDown(){
    return intakeController.getLeftBumperPressed();
}


/** 
 * @return boolean
 */
//OTHER THINGS
// STOP arm
public boolean stopArm(){
    return intakeController.getStartButton();
}


}
