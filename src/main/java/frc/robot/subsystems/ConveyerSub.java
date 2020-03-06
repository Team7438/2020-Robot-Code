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
public class ConveyerSub extends Subsystem {
    public static VictorSPX conveyer1 = new VictorSPX(RobotMap.Conveyer1);
    public static VictorSPX conveyer2 = new VictorSPX(RobotMap.Conveyer2);

  public static void Rollin() {
    conveyer1.set(ControlMode.PercentOutput, -0.15);
    conveyer2.set(ControlMode.PercentOutput, 0.15);
  }

  public static void Rollout(){
    conveyer1.set(ControlMode.PercentOutput, 0.15);
    conveyer2.set(ControlMode.PercentOutput, -0.15);
  }

  public static void RollStop(){
    conveyer1.set(ControlMode.PercentOutput, 0.0);
    conveyer2.set(ControlMode.PercentOutput, 0.0);
  }

  // Add jam function
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}