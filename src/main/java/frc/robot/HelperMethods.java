package frc.robot;

public final class HelperMethods {



    public static double limitValInRange(double min, double max, double value) {
        if (value > max) {return max;}
        else if (value < min) {return min;}
        else {return value;}
    }



    
}
