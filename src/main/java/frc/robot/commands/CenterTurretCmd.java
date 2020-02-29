/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.TurretSub;

public class CenterTurretCmd extends Command {


  public CenterTurretCmd() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (SmartDashboard.getNumber("TurretDistance", 132) < 60) {
      if (SmartDashboard.getNumber("TurretDistance", 132) < 130) {
        TurretSub.setPower(-0.1);
      } else {
        TurretSub.setPower(-0.4);
      }
    } else if (SmartDashboard.getNumber("TurretDistance", 132) > 135) {
      if (SmartDashboard.getNumber("TurretDistance", 132) > 200) {
        TurretSub.setPower(0.4);
      } else {
        TurretSub.setPower(0.1);
      }
    } else {
      TurretSub.setPower(0);
    }
  } 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (SmartDashboard.getNumber("TurretDistance", 132) < 134 && SmartDashboard.getNumber("TurretDistance", 132) > 130) {
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