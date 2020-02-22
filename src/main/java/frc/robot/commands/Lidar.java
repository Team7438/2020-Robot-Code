/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.LIDARLite;

public class Lidar extends Command {
  public Lidar() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      LIDARLite.startMeasuring();
      System.out.println("STARTING");
      
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // int distance = LIDARLite.getDistance();
    System.out.println("Distance" + LIDARLite.getDistance());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    LIDARLite.stopMeasuring();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
