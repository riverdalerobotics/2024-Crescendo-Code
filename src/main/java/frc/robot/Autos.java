// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Autos {
   
   
    // public Command podiumSubwooferTwoNotes(){
    //     return new PathPlannerAuto("2 note podium subwoofer");
    // }
    // public Command ampSubwooferTwoNotes(){
    //     return new PathPlannerAuto("2 note amp subwoofer");
    // }
    
    

    public Command test(){
        return new PathPlannerAuto("1st test");
    }
    public Command testTwo(){
        return new PathPlannerAuto("2nd test");
    }
    public Command testThree(){
        return new PathPlannerAuto("3rd test");
    } 
    public Command testFour(){
        return new PathPlannerAuto("4th test");
    }
    public Command testFive(){
        return new PathPlannerAuto("5th test");
    }
    
    
    public Command leaveAfterShootingPodiumSide(){
        return new PathPlannerAuto("Leave after shooting podium");
    }

    public Command doNothing(){
        return new PathPlannerAuto("Do Nothing");
    }
    
    public Command mobilityWithStyle(){
        return new PathPlannerAuto("Mobility with style");
    }
    public Command outOfWayMobility(){
        return new PathPlannerAuto("Mobility GET OUT OF THE WAY");
    }

    public Command midSubWooferFourNotesAmpFirst(){
        return new PathPlannerAuto("4 note mid subwoofer (AMP FIRST)");
    }

    public Command midSubWooferFourNotesPodiumFirst(){
        return new PathPlannerAuto("4 note mid subwoofer (PODIUM FIRST)");
    }
    public Command midSubwooferFourNotesActual(){
        return new PathPlannerAuto("4 note Final Mid Subwoofer");
    }

    public Command podiumSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note podium subwoofer");
    }
    public Command ampSubwooferTwoNotes(){
        return new PathPlannerAuto("2 note amp subwoofer");
    }

    public Command shootAndStop() {
        return new PathPlannerAuto("shoot and stop");
    }


    public Command queenAOne() {
        return new PathPlannerAuto("QA1");
    }
    public Command queenATwo() {
        return new PathPlannerAuto("QA2");
    }
    public Command queenAThree() {
        return new PathPlannerAuto("QA3");
    }
    public Command queenAFour() {
        return new PathPlannerAuto("QA4");
    }


}
