// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



/* --------------------------------------------------------------------------- *
 * This class was created with the purpose of simplifying the configuration    *
 * process of the TalonFX objects in WPI. It also adds the functionality to    *
 * check if the motor has reached a desired position/velocity setpoint.        *
 * Please also view the TalonHelper class for other methods used to simplify   *
 * the TalonFX configuration process                                           *
 * This class was created on March 15th, 2024                                  *
 * --------------------------------------------------------------------------- */
 
package frc.robot.R3P2CustomClasses;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class P2TalonFX extends TalonFX {
    double tolerance;

    public P2TalonFX(int port) {
        super(port);
    }

    /**
     * Sets the tolerance for closed loop control
     * <p>
     * The unit of this variable is whatever the unit of the closed loop controller is
     * @param setTolerance The desired tolerance (the unit will be converted to the unit of the Talon encoder)
     */
    public void setTolerance(double setTolerance) {
        tolerance = setTolerance;
    }



    //IMPORTANT METHOD -LIAM
    
       /**
     * Configures the talon using a created TalonFXConfiguration object. This method simplifies the process of configuring the motor.
     * <p>
     * Config is reset to default first to avoid any unintenional configurations passing over between power cycles
     * @param config The TalonFXConfiguration object returned by createTalonConfig method in TalonHelper.java
     */
    public void config(TalonFXConfiguration config) {
        //resets to factory default
        this.getConfigurator().apply(new TalonFXConfiguration());
        this.getConfigurator().apply(config, 0.050);

    }

    /**
     * Used to see if a TalonFX motor has reached a desired position setpoint.
     * <p>
     * All position control modes should use this method
     * @param setpoint Desired setpoint (get this value from the control mode affecting the motor)
     * @return true if the motor is within tolerance of the set position
     */
    public boolean atSetpointPosition(double setpoint) {
        double motorPosition = this.getPosition().getValueAsDouble();
    

        if (motorPosition <= setpoint && motorPosition + tolerance >= setpoint) {
            //System.out.println("end low");
            return true;
        }
        else if (motorPosition >= setpoint && motorPosition - tolerance <= setpoint) {
            //System.out.println("end high");
            return true;
        }
        else {
            //System.out.println("continue");
            return false;
        }
    }

    /**
     * Used to see if a TalonFX motor has reached a desired velocity setpoint
     * <p>
     * All velocity control modes should use this method
     * @param setpoint Desired setpoint (get this value from the control mode affecting the motor)
     * @return true if the motor is within tolerance of the set velocity
     */
    public boolean atSetpointVelocity(double setpoint) {
        this.getControlMode();
        double motorVelocity = this.getVelocity().getValueAsDouble();
        if (motorVelocity <= setpoint && motorVelocity + tolerance >= setpoint) {
            return true;
        }
        else if (motorVelocity >= setpoint && motorVelocity - tolerance <= setpoint) {
            return true;
        }
        else {
            return false;
        }
    }
}