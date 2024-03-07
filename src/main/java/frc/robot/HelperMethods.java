package frc.robot;

import frc.robot.Constants.OperatorConstants;

public final class HelperMethods {

    /**
     * Limits a value between a range defined by a min and max value
     * @param min the minimum value returned by the method
     * @param max the maximum value returned by the method
     * @param value the value being adjusted to fit within the range
     * @return The value adjusted to fit within the defined range
     */
    public static double limitValInRange(double min, double max, double value) {
        if (value > max) {return max;}
        else if (value < min) {return min;}
        else {return value;}
    }



    /**
     * Many of our controllers have issues with deadbands applying a small power to the motors at the joystick 0 position.
     * All joystick inputs are passed through this method to ensure they don't have a small value when zero'd
     * @param controllerInput the raw controller input
     * @return raw controller input if above deadband. 0 if below
     */
    public static double applyInputDeadband(double controllerInput) {
        if (controllerInput < OperatorConstants.kControllerDeadbandValue && controllerInput > -OperatorConstants.kControllerDeadbandValue) {
            return 0;
        }
        else {
            return controllerInput;
        }
    }


    
}
