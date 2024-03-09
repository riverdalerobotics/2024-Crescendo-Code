// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Autos {
   
   
    public Command podiumSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note podium subwoofer");
    }
    public Command ampSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note amp subwoofer");
    }
    public Command midSubWooferFourNotes(){
        return new PathPlannerAuto("4 note mid subwoofer");
    }
    public Command doNothing(){
        return new PathPlannerAuto("Do Nothing");
    }
    public Command shootOnly(){
        return new PathPlannerAuto(null);
    }
    public Command test(){
        return new PathPlannerAuto("1st test");
    }
    public Command testTwo(){
        return new PathPlannerAuto("2nd test");
    }
    public Command testThree(){
        return new PathPlannerAuto("3rd test");
    }
    public Command mobilityWithStyle(){
        return new PathPlannerAuto("Mobility with style");
    }
    public Command outOfWayMobility(){
        return new PathPlannerAuto("Mobility GET OUT OF THE WAY");
    }


    public Command weirdPodiumSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note podium subwoofer (new)");
    }
    public Command weirdAmpSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note amp subwoofer (new)");
    }
    public Command weirdMidSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note mid subwoofer (new)");
    }
    public Command shootAndStop() {
        return new PathPlannerAuto("shoot and stop");
    }



}
