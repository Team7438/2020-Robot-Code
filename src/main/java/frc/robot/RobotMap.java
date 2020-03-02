/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {

  //Drive Motors
  public static int left1 = 3;
  public static int left2 = 2;
  public static int right1 = 0;
  public static int right2 = 1;
  
  //Intake Motors
  public static int rearIntake = 5;

  //Turret
  public static int turretYaw = 8;

  //Conveyer Motors
  public static int Conveyer1 = 52;
  public static int Conveyer2 = 53;

  //Climb Motors
  public static int climbFront = 6;

  //Elevator Motor
  public static int elevator = 7;

  //Drive Encoders
  public static int rightEncoderPort1 = 0;
  public static int rightEncoderPort2 = 1;
  public static int leftEncoderPort1 = 2;
  public static int leftEncoderPort2 = 3;

  //Elevator Winch Encoder
  public static int elevatorEncoderPort1 = 4;
  public static int elevatorEncoderPort2 = 5;


  //Solenoid Valves for Pneumatics
  public static int eleSole1 = 1;
  public static int eleSole2 = 0;
  public static int lifterSole3 = 3; //Ball Lifter Piston
}
