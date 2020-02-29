/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LoaderSub;

import java.util.concurrent.TimeUnit;

public class LoadCmd extends Command {

  private static Boolean hasRun = false;

  public LoadCmd() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.loader);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hasRun = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!hasRun) {
      Robot.loader.liftUp();
      try {
        TimeUnit.SECONDS.sleep(1);
        Robot.loader.liftDown();
        hasRun = true;
      } catch (InterruptedException e) {
        Robot.loader.liftDown();
        hasRun = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (hasRun) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.loader.liftDown();
  }
}
