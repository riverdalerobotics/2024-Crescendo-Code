// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



//The auto code in this file looks very confusing, but it's not actually too bad. All it does is put options on smart dashboard that drivers can select 
//before the match. Depending on which option was selected, a different auto will run. 


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected;
  private static final String shootOnly = "Shoot and then do Nothing"; //centennial comp win

  private static final String doNothingLol = "DO NOTHING";
  private static final String test = "curve path 1.6764 meters down, 155 inches to the right"; // max velocity is 1 m/s
  private static final String testTwo = "straight line goes 155 inches or 3.937 meters"; //max velocity is 1.1 m/s
  private static final String testThree = "rotate 180 degrees moving 3.937 meters"; //max velocity is 3 m/s
  private static final String testFour = "rotate 180 degrees while going forward and back"; //max velocity is 3 m/s
  private static final String testFive = "pickup close mid slowly (test wth commands)";
  
  private static final String mobilityWithStyle = "go 2.26 meters, rotate 180 degrees" ; //max velocity is 1.5 m/s
  private static final String leaveAfterShootingPodium = "Windsor lol";

  private static final String podiumSubwooferTwoNotes = "Podium SIde Subwoofer Shoot And Rretrieve Podium Note and Shoot (different Intake and Shoot side)"; 
  private static final String ampSubwooferTwoNotes = "Amp Side Subwoofer Shoot and Retrieve Amp Note and Shoot (different Intake and Shoot side)";
 
  private static final String midSubwooferFourNotesAmpFirst = "Middle Side Subwoofer Shoot and Retrieve and Shoot 3 close notes (amp note first) (different Intake and Shoot side)";
  private static final String midSubwooferFourNotesPodiumFirst = "Middle Side Subwoofer Shoot and Retrieve and Shoot 3 close notes (podium note first) (different Intake and Shoot side)";
  private static final String midSubwooferFourNotesActual = "Middle Side Suwboofer Shoot and Retrieve and Shoot 3 close notes asap (mid, then podium, then amp)";
  
  private static final String shootAndStop = "Any side start - shoot note into speakker";

  private static final String queenAOne = "1";
  private static final String queenATwo = "2";
  private static final String queenAThree = "3";
  private static final String queenAFour = "4";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(); 


   
    }
  

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //we always want our LED's to update as long as the robot is on, so we call this in robotperiodic
    m_robotContainer.LED.setLEDColor();
    m_robotContainer.LED.updateAutoAlignLED();
  } 

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}


  public void enabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {


    //When auto begins, this large statement selects the autonomous command based on what was selected in smart dashboard
    //switch statements just act like big if else statements
     
  }

  /** This function is called atheriodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
