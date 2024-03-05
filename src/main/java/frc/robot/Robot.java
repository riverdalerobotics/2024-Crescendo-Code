// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private static final String shootOnly = "Shoot and then do Nothing";
  private static final String podiumSubwooferTwoNotes = "Podium Side Subwoofer Shoot And Retrieve Podium Note and Shoot";
  private static final String ampSubwooferTwoNotes = "Amp Side Subwoofer Shoot and Retrieve Amp Note and Shoot";
  private static final String midSubwooferFourNotes = "Middle Side Subwoofer Shoot and Retrieve and Shoot 3 close notest";
  private static final String doNothingLol = "DO NOTHING";
  private static final String test = "curve path 1.6764 meters down, 6.35 meters to the right"; // max velocity is 1 m/s
  private static final String testTwo = "straight line goes 155 inches or 3.937 meters"; //max velocity is 1.1 m/s
  private static final String testThree = "rotate 180 degrees moving 3.937 meters"; //max velocity is 3 m/s

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(); 

    m_chooser.setDefaultOption("SHOOT ONLY", shootOnly);
    m_chooser.addOption("1+1 PODIUM Side Subwoofer", podiumSubwooferTwoNotes);
    m_chooser.addOption("1+1 AMP side Subwoofer", ampSubwooferTwoNotes);
    m_chooser.addOption("1+3 MID side Subwoofer", midSubwooferFourNotes);
    m_chooser.addOption("DO NOTHING", doNothingLol);
    m_chooser.addOption("test123", test);
    m_chooser.addOption("2nd test", testTwo);
    m_chooser.addOption("3rd test", ampSubwooferTwoNotes);
  
    SmartDashboard.putData("Auto choices", m_chooser);
   
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

     m_autoSelected = m_chooser.getSelected();
      System.out.println("Auto selected: " + m_autoSelected);
      /* */
     
      switch (m_autoSelected) {
        case shootOnly:
          m_autonomousCommand = m_robotContainer.getShootOnlyAuto();
          break;
  
        case podiumSubwooferTwoNotes:
          m_autonomousCommand = m_robotContainer.getPodiumSubwooferTwoNotesAuto();
          break;
  
        case ampSubwooferTwoNotes:
          m_autonomousCommand = m_robotContainer.getAmpSubwooferTwoNotesAuto();
          break;  
          
        case midSubwooferFourNotes:
          m_autonomousCommand = m_robotContainer.getMidSubwooferFourNotesAuto();
          break;
        case doNothingLol:
          m_autonomousCommand = m_robotContainer.getDoNothingAuto();
          break;
        case test:
          m_autonomousCommand = m_robotContainer.getTestAuto();
          break;
        case testTwo: 
          m_autonomousCommand = m_robotContainer.getTestSecondAuto();
          break;
        case testThree: 
          m_autonomousCommand = m_robotContainer.getTestThreeAuto();
          break;
        

        //REDUNDANT, I think we can delete maybe 
        default:
          m_autonomousCommand = m_robotContainer.getShootOnlyAuto();
          break;
  
      }
    
      // schedule the autonomous command
      if (m_autonomousCommand != null) {
        System.out.println("Successfully scheduled");
        m_autonomousCommand.schedule();
      }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.CHASSIS.straightenModules();
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
