package frc.robot;

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



    
}
