
package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;


public class TalonHelper {


    public TalonHelper() {

    }

    

    /**
     * Creates and returns a talon configuration object. Used to simplify the creation process
     * @param kP Proportional term of the controller
     * @param kI Integral term of the controller
     * @param kD Derivative term of the controller
     * @param kV FeedForward term of the controller, should be the voltage/duty cycle need to move at 1 unit per second of your control type
     * @param kS Static FeedForward, the voltage/duty cycle needed to beat static friction in your system
     * @param MMCruiseVelocity The max velocity acheivable by motion magic (velocity motion magic controllers ignore this setting)
     * @param MMAcceleration The max acceleration acheivable by motion magic
     * @param MMJerk The max jerk acheviable by motion magic (jerk is the rate of increase of acceleration per second)
     * @param currentLimit the stator current limit of the motor (stator/output current limits are meant to protect the motor)
     * @param closedLoopRamp the time it takes in seconds to ramp up to speeds in closed loop (PID) control. Current unused as I am unsure how this interacts with motion magic
     * @param gearRatio the gear ratio of the mechanism (how many rotations the mechanism makes when the motor turns once)
     * @param peakDutyCycle the max output closed loop control can supply to the mechanism
     * @return A TalonFXConfiguration object that can be passed into the configTalon method with a motor to configure it
     */
    public static TalonFXConfiguration createTalonConfig(double kP, double kI, double kD, double kV, double kS, double MMCruiseVelocity, double MMAcceleration, double MMJerk, double currentLimit, double closedLoopRamp, double gearRatio, double peakDutyCycle) {
        var talonFXConfigs = new TalonFXConfiguration();


        //TODO: Apply the velocity factor in the set velocity method
        talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio;

    
        

        var slot0Config = talonFXConfigs.Slot0;
        slot0Config.kS = kS;
        //TODO: Find voltage required maintain position at the horizontal
        slot0Config.kV = kV;
        slot0Config.kP = kP;
        slot0Config.kI = kI;
        slot0Config.kD = kD;

        //TODO: Figure out how to apply gear ratio conversion factor to the internal encoders (currently only measuring rotations when it should be degrees)

        /**Motion magic is a form of motion profiling offered by CTRE
        Read this page for more information on what motion profiling is: https://docs.wpilib.org/en/stable/docs/software/commandbased/profile-subsystems-commands.html
        In short, it gradually raises the desired setpoint, instead of abruptly changing the set point,
        resulting in a smoother motion with fewer voltage/current spikes */
        var motionMagicConfig = talonFXConfigs.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = MMCruiseVelocity; //no limit on max velocity in degrees
        motionMagicConfig.MotionMagicAcceleration = MMAcceleration; // limit of 160 degrees/s acceleration
        motionMagicConfig.MotionMagicJerk = MMJerk; // limit of 700 degrees/s^2 jerk (limits acceleration)
        
        //Sets the current limit of our intake to ensure we don't explode our motors (which is bad)
        var currentConfig = talonFXConfigs.CurrentLimits;
        currentConfig.StatorCurrentLimit = currentLimit;
        currentConfig.StatorCurrentLimitEnable = true;


        //This is not currently used, but may be useful for avoiding voltage spikes
        var closedLoopRampsConfig = talonFXConfigs.ClosedLoopRamps;
        closedLoopRampsConfig.VoltageClosedLoopRampPeriod = closedLoopRamp;


        var motorOutputConfig = talonFXConfigs.MotorOutput;
        motorOutputConfig.PeakForwardDutyCycle = peakDutyCycle;
        motorOutputConfig.PeakReverseDutyCycle = -peakDutyCycle;

        return talonFXConfigs;
    }



    /**
     * Creates and returns a talon configuration object. Used to simplify the creation process
     * @param kP Proportional term of the controller
     * @param kI Integral term of the controller
     * @param kD Derivative term of the controller
     * @param kV FeedForward term of the controller, should be the voltage/duty cycle need to move at 1 unit per second of your control type
     * @param kS Static FeedForward, the voltage/duty cycle needed to beat static friction in your system
     * @param MMCruiseVelocity The max velocity acheivable by motion magic (velocity motion magic controllers ignore this setting)
     * @param MMAcceleration The max acceleration acheivable by motion magic
     * @param MMJerk The max jerk acheviable by motion magic (jerk is the rate of increase of acceleration per second)
     * @param currentLimit the stator current limit of the motor (stator/output current limits are meant to protect the motor)
     * @param closedLoopRamp the time it takes in seconds to ramp up to speeds in closed loop (PID) control. Current unused as I am unsure how this interacts with motion magic
     * @param gearRatio the gear ratio of the mechanism (how many rotations the mechanism makes when the motor turns once)
     * @param peakDutyCycle the max output closed loop control can supply to the mechanism
     * @param gravityType the type of gravity affecting the system. This is used for elevator or arm systems and affects how the closed loop controller applies feed forward
     * An arm will have the highest feed forward value when it is completely horizontal, as that position requires the most power to hold
     * @return A TalonFXConfiguration object that can be passed into the configTalon method with a motor to configure it
     */
    public static TalonFXConfiguration createTalonConfig(double kP, double kI, double kD, double kV, double kS, double MMCruiseVelocity, double MMAcceleration, double MMJerk, double currentLimit, double closedLoopRamp, double gearRatio, double peakDutyCycle, GravityTypeValue gravityType) {
        var talonFXConfigs = new TalonFXConfiguration();


        //TODO: Apply the velocity factor in the set velocity method
        talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio;
        

        var slot0Config = talonFXConfigs.Slot0;
        slot0Config.kS = kS;
        //TODO: Find voltage required maintain position at the horizontal
        slot0Config.kV = kV;
        slot0Config.kP = kP;
        slot0Config.kI = kI;
        slot0Config.kD = kD;

        //TODO: change the 0 encoder position of the arm to be horizontal with the ground

        //This tells the system that the controller is affecting an arm.
        //This changes how to feedforward values are applied to the system.
        //The highest voltage is needed when the arm is horizontal, and the lowest is needed when the arm is vertical
        //This voltage requirement follows a cosine ratio
        slot0Config.GravityType = gravityType;


        //TODO: Figure out how to apply gear ratio conversion factor to the internal encoders (currently only measuring rotations when it should be degrees)

        /**Motion magic is a form of motion profiling offered by CTRE
        Read this page for more information on what motion profiling is: https://docs.wpilib.org/en/stable/docs/software/commandbased/profile-subsystems-commands.html
        In short, it gradually raises the desired setpoint, instead of abruptly changing the set point,
        resulting in a smoother motion with fewer voltage/current spikes */
        var motionMagicConfig = talonFXConfigs.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = MMCruiseVelocity; //no limit on max velocity in degrees
        motionMagicConfig.MotionMagicAcceleration = MMAcceleration; // limit of 160 degrees/s acceleration
        motionMagicConfig.MotionMagicJerk = MMJerk; // limit of 700 degrees/s^2 jerk (limits acceleration)

        //Sets the current limit of our intake to ensure we don't explode our motors (which is bad)
        var currentConfig = talonFXConfigs.CurrentLimits;
        currentConfig.StatorCurrentLimit = currentLimit;
        currentConfig.StatorCurrentLimitEnable = true;


        //This is not currently used, but may be useful for avoiding voltage spikes
        var closedLoopRampsConfig = talonFXConfigs.ClosedLoopRamps;
        closedLoopRampsConfig.VoltageClosedLoopRampPeriod = closedLoopRamp;

        var motorOutputConfig = talonFXConfigs.MotorOutput;
        motorOutputConfig.PeakForwardDutyCycle = peakDutyCycle;
        motorOutputConfig.PeakReverseDutyCycle = -peakDutyCycle;

        return talonFXConfigs;
    }
}
