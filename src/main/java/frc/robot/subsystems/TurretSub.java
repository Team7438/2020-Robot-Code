/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 */


public class TurretSub extends Subsystem {
    public static VictorSPX turretYaw = new VictorSPX(RobotMap.turretYaw);
    public static Joystick blackButtons = new Joystick(4);
  // Turret speed
  private static double turretSpeed = 0.29;

  public static void ManualControlYaw() {
    if (blackButtons.getRawAxis(0) >= 0.9) {
      RotateRight();
    } else if (blackButtons.getRawAxis(0) <= -0.9) {
      RotateLeft();
    } else {
      stopRotate();
    }
  }

  public static void RotateLeft() {
    if (SmartDashboard.getNumber("TurretDistance", 0) <= -730) {
      turretYaw.set(ControlMode.PercentOutput, 0);
    } else {
      turretYaw.set(ControlMode.PercentOutput, turretSpeed);
    }
  }

  public static void RotateRight() {
    if (SmartDashboard.getNumber("TurretDistance", 0) >= 1110) {
      turretYaw.set(ControlMode.PercentOutput, 0);
    } else {
      turretYaw.set(ControlMode.PercentOutput, -turretSpeed);
    }
  }

  public static void setPower(double d) {
    turretYaw.set(ControlMode.PercentOutput, d);

    if (SmartDashboard.getNumber("TurretDistance", -100000) <= -730) {
      if (d < 0) {
        turretYaw.set(ControlMode.PercentOutput, d);
      } else if (d > 0) {
        turretYaw.set(ControlMode.PercentOutput, 0);
      } else {
        turretYaw.set(ControlMode.PercentOutput, 0);
      }
    } else if (SmartDashboard.getNumber("TurretDistance", 100000) >= 1110) {
      if (d < 0) {
        turretYaw.set(ControlMode.PercentOutput, 0);
      } else if (d > 0) {
        turretYaw.set(ControlMode.PercentOutput, d);
      } else {
        turretYaw.set(ControlMode.PercentOutput, 0);
      }
    }

  }

  public static void stopRotate() {
    turretYaw.set(ControlMode.PercentOutput, 0);
  }

  

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
} 
