// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivotCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class NewAutoPivotToAngle extends Command {
  /** Creates a new AutoPivotToAngle. */
  double tolerance;
  double desiredAngle;
  PivotSubsystem pivot;
  double maxCurrent = PivotConstants.kHardStopCurrentThreshold;
  double hardStopPosition = PivotConstants.PIDConstants.kMinSetpoint;
  boolean considerGravity = false;
  boolean gravOffsetAcheived = true;
  double gravityAngle;
  public NewAutoPivotToAngle(double angle, PivotSubsystem pivotSubsystem, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivotSubsystem;
    this.desiredAngle = angle;
    this.tolerance = tolerance;
    addRequirements(pivot);
  }

  /**
   * Alternative constructor which tells the arm to enable gravity consideration
   * <p>
   * The arm has trouble when moving against gravity. This code will detect when the arm is moving to a point
   * that fights against gravity, and will add 10 degrees to the angle, then move back to the original desired angle
   * once it is above the angle and can work with gravity
   * @param angle
   * @param pivotSubsystem
   * @param tolerance
   * @param considerGrav
   */
  public NewAutoPivotToAngle(double angle, PivotSubsystem pivotSubsystem, double tolerance, boolean considerGrav) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivotSubsystem;
    this.desiredAngle = angle;
    this.tolerance = tolerance;
    this.considerGravity = considerGrav;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //The constant tolerance value is in degrees, but the internal controller uses rotations.
    //We convert the degrees tolerance into rotations so the internal motor controller can work with the value
    pivot.setPivotTolerance(Units.degreesToRotations(tolerance));
    pivot.specCommandRunning = true;
    pivot.setPivotAngleDegrees(desiredAngle);


    //Checks if the angle should be adjusted initially to account for gravity
    //When fighting against significant gravity, the arm will initially aim for an angle above the desired angle,
    //then pivot down once it is above the desired position, using gravity to help its movement
    //To view a visualization of what angles are adjusted for gravity, please view this imgur diagram
    //https://imgur.com/a/vFEDG7T
    double curAngle = pivot.getEncoders();
    if (considerGravity) {
      if(curAngle > -75) {
        if(desiredAngle < curAngle && desiredAngle > -75) {
          gravityAngle = desiredAngle - 15;
          pivot.setPivotAngleDegrees(gravityAngle);
          pivot.setPivotTolerance(Units.degreesToRotations(PivotConstants.PIDConstants.kGravityOffsetTolerance));
          gravOffsetAcheived = false;
          for(int i = 0; i < 10; i++) {
          System.out.println("FRONT GRAV");
          }
        }
      }
      else if(curAngle < -105) {
        if(desiredAngle > curAngle && desiredAngle < -105) {
          //activate grav accounting (add negative angle value)
          gravityAngle = desiredAngle + 15;
          pivot.setPivotAngleDegrees(gravityAngle);
          pivot.setPivotTolerance(Units.degreesToRotations(PivotConstants.PIDConstants.kGravityOffsetTolerance));
          gravOffsetAcheived = false;
          for(int i = 0; i < 10; i++) {
          System.out.println("BACK GRAV");
          }
        }
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override  
  public void execute() {
  
    //TODO: Fix the logic for this because it kind of sucks rn (if encoder is off the correct value this will not work)
    if (pivot.getCurrent() > maxCurrent){
      pivot.setPivotAngleDegrees(hardStopPosition);
    }

  
    if (gravOffsetAcheived == false) {
      if(pivot.getPivot1().atSetpointPosition(Units.degreesToRotations(gravityAngle))) {

        //Once the angle above the desired angle is reached, begin moving towards the actual setpoint with an adjusted tolerance for more accuracy
        gravOffsetAcheived = true;
        pivot.setPivotTolerance(Units.degreesToRotations(tolerance));
        pivot.setPivotAngleDegrees(desiredAngle);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gravityAngle = 0;
    
    gravOffsetAcheived = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //This is a custom method implemented in our P2TalonFX class. The setpoint must be passed in,
    //then the motor will check if it has reached it within its range of tolerance
    //Grav offset must be checked as otherwise the command would end unexpectedly 
    return pivot.getPivot1().atSetpointPosition(Units.degreesToRotations(desiredAngle)) && gravOffsetAcheived;
  }
}