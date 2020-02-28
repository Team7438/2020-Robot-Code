/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 */

public class TurretSub extends Subsystem {
    public static VictorSPX turretYaw = new VictorSPX(RobotMap.turretYaw);

  public static void RotateLeft() {
    turretYaw.set(ControlMode.PercentOutput, 0.2);
  }

  public static void RotateRight() {
    turretYaw.set(ControlMode.PercentOutput, -0.2);
  }

  public static void RotateTo(Integer degrees) {

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