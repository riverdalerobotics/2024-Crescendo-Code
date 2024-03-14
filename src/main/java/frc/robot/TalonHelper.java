
package frc.robot;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
public class TalonHelper {


    public TalonHelper() {

    }


    public static void configTalon(TalonFX motor, TalonFXConfiguration config) {
        //resets to factory default
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.getConfigurator().apply(config, 0.050);

    }
    

    public static TalonFXConfiguration createTalonConfig(double kP, double kI, double kD, double kV, double kS, double MMCruiseVelocity, double MMAcceleration, double MMJerk, double currentLimit, double closedLoopRamp, double gearRatio) {
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


        return talonFXConfigs;
    }

    public static TalonFXConfiguration createTalonConfig(double kP, double kI, double kD, double kV, double kS, double MMCruiseVelocity, double MMAcceleration, double MMJerk, double currentLimit, double closedLoopRamp, double gearRatio, GravityTypeValue gravityType) {
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


        return talonFXConfigs;
    }
}
