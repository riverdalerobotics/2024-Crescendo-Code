// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    XboxController moveController = new XboxController(0);
    XboxController intakeController = new XboxController(1);

/** 
 * @return double
 */
//Movement controls

//x is fwd backward
public double xSpeed(){
    return moveController.getLeftY();
}

//y is left right
public double ySpeed(){
    return moveController.getLeftX();
}

public double rotate(){
    return moveController.getRightX();
}

//Go to pickup position
public boolean pickUpPos(){
    return intakeController.getLeftBumper();
}

//Go to shoot position
public boolean shootPos(){
    return intakeController.getRightBumper();
}
public double beltSpeed(){
    return intakeController.getRightY();
}
public double shooterSpeed(){
    return intakeController.getRightTriggerAxis();
}
//Shoot
//TODO: make this a boton!! it is now in "testing mode"
public double manShoot(){
    return intakeController.getRightTriggerAxis();
}
public boolean shoot(){
    return intakeController.getAButtonPressed();
}
//Go to drivePos
public boolean drivePos(){
    return intakeController.getLeftStickButton();
}

// Arm Up and down
public boolean moveArmUp(){
    return intakeController.getLeftX();
}
//togle feild oriented
public boolean toggleFieldOriented(){
    return moveController.getBackButton();
}

public boolean toggleSlowMode() {
    return moveController.getStartButton();
}

//OTHER THINGS
// STOP arm
public boolean stopArm(){
    return intakeController.getStartButton();
}

public boolean resetGyro(){
    return intakeController.getBackButton();
}

}
