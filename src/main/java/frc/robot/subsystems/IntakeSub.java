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
public class IntakeSub extends Subsystem {
    public static VictorSPX intake = new VictorSPX(RobotMap.rearIntake);
    private static Boolean isRollingIn = false;



  public static void Rollin() {
    intake.set(ControlMode.PercentOutput, 0.50);
    isRollingIn = true;
  }

  public void Rollout(){
    intake.set(ControlMode.PercentOutput, -0.50);
  }

  public static void RollStop(){
    intake.set(ControlMode.PercentOutput, 0);
    isRollingIn = false;
  }

  public static Boolean getRollingInStatus() {
      return isRollingIn;
  }

  // Add jam function
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}