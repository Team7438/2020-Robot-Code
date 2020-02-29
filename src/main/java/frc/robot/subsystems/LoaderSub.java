/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
/**
 * Add your docs here.
 */
public class LoaderSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static Solenoid oneLifter = new Solenoid(11, RobotMap.lifterSole3);

  public static void liftUp() {
    oneLifter.set(true);
   }

  //public boolean lifted(){
  //  if(Robot.loader.oneLifter.get() == true) {
  //    return true;
  //  }else{
  //    return false;
  //  }
//}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

public static void liftDown() {
    oneLifter.set(false);
  }
}
