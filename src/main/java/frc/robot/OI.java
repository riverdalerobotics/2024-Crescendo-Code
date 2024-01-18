// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    XboxController moveController = new XboxController(0);
    XboxController intakeController = new XboxController(1);
//Movement controls
public double xSpeed(){
    return moveController.getLeftX();
}

public double ySpeed(){
    return moveController.getLeftY();
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

//Shoot
//TODO: make this a boton!! it is now in "testing mode"
public double shoot(){
    return intakeController.getRightTriggerAxis();
}

//Go to drivePos
public boolean drivePos(){
    return intakeController.getLeftStickButton();
}

// Arm Up and down
public double moveArm(){
    return intakeController.getLeftY();
}
//togle feild oriented
public boolean togFeildOriented(){
    return moveController.getBackButton();
}
//OTHER THINGS
// STOP arm
public boolean stopArm(){
    return intakeController.getStartButton();
}

// Todao botton
public boolean stopDrive(){
    return moveController.getStartButton();
}

public boolean resetGyro(){
    return intakeController.getBackButton();
}

}
