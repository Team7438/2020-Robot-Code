/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSub;
import frc.robot.commands.CenterTurretCmd;

public class TurretSearchCmd extends Command {

  private static Boolean rotatingLeft = false;

  public TurretSearchCmd() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("Potato");
    if (rotatingLeft) {
      TurretSub.setPower(0.2);
    } else if (!rotatingLeft) {
      TurretSub.setPower(-0.2);
    } else {
      TurretSub.setPower(0);
    }
    if (SmartDashboard.getNumber("TurretDistance", 0) <= -604) {
      rotatingLeft = false;
    } else if (SmartDashboard.getNumber("TurretDistance", 0) >= 1007) {
      rotatingLeft = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (SmartDashboard.getBoolean("isValid", false)) {
      return true;
    } else {
      return false;
    }
    //return Robot.limitSwitch.get();
    //return Robot.cargoLoader.isSwitchSet();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    TurretSub.stopRotate();
    // Console.WriteLine("Text to print");

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    TurretSub.stopRotate();
  }
}