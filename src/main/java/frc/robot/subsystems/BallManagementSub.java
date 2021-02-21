// +-------------------------------------------------------------------+
// |                         Sensor Locations                          |
// +-------------------------------------------------------------------+

//                                                                Intake
//
//                                                                 +--+
//                                                                 |--|
//                                                                 |--|
//                                                                 |--|
//                  Preload                  Conveyer              |--|
//                                                                 |--|
//               +-----------+-------------------------------------|--|
//               |           |                                     |--|
//               |           |                                     |--|
//               |           |                                     |--|
//               | ^         |  ^          ^          ^          ^ |--|
//               | |         |  |          |          |          | |--|
//               | |         |  |          |          |          | |--|
//               +-----------+-------------------------------------|--|
//                 |            |          |          |          | |--|
//                 |            |          |          |          | |--|
//                 +            +          +          +          + |--|
//                 5            4          3          2          1 |--|
//                                                                 |--|
//                                 Sensor Locations                +--+

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */

public class BallManagementSub extends Subsystem {

  public static Solenoid shooterSole = new Solenoid(11, RobotMap.shooterSole2);
  public static VictorSPX turretPitch = new VictorSPX(RobotMap.turretPitch);

  public static TalonSRX shooterWheel1 = new TalonSRX(RobotMap.shooterWheel1);
  public static TalonSRX shooterWheel2 = new TalonSRX(RobotMap.shooterWheel2);

  public final static DutyCycleEncoder turret_dutyCycleEncoder = new DutyCycleEncoder(9);
  public static Joystick blackButtons = new Joystick(1);

  public Boolean isForward = true;
  public static Double tempPower = 0.0;
  public static Double power = 0.0;
  public static Double currentEncoder = 0.0;
  public static Double powerToReturn = 0.0;
  public static Boolean flipped = false;

  // 50 loop units per second
  // 4 EU per 50 LU
  // 0.08 EU per LU

  // Get current EV
  // Compare to lastEncoder
  // If it's less than, increase power.
  // If it's greater than, decrease power.
  // Starting power 0.1

  public static void ManualControlPitch() {
    if (blackButtons.getRawAxis(1) >= 0.9) {
      pitchDown();
    } else if (blackButtons.getRawAxis(1) <= -0.9) {
      pitchUp();
    } else {
      stopPitch();
    }
  }

  public static void setUpEncoder() {
    turret_dutyCycleEncoder.setDistancePerRotation(50);
    turret_dutyCycleEncoder.reset();
  }


  public static void shooterPistonOut() {
    shooterSole.set(true);
  }

  public static void shooterPistonIn() {
    shooterSole.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static Double getEncoderValue() {
    return currentEncoder = turret_dutyCycleEncoder.getDistance() + 1000;
    //Returns the turret pitch encoder value, adjusted to always be positive.
  }

  public static void pitchUp() {
    if (!flipped) {
      turretPitch.set(ControlMode.PercentOutput, -0.7);
    } else {
      turretPitch.set(ControlMode.PercentOutput, -0.1);
    }
  
}

public static void pitchDown() {
  if (!flipped) {
    turretPitch.set(ControlMode.PercentOutput, 0.1);
  } else {
    turretPitch.set(ControlMode.PercentOutput, 0.7);
  }
}

public static void goToPosition(Integer position) {
  //Pitch up until overshoot, then, slowly move down. Allow large margin of error
  switch (position) {
    case 1:
      //Lower dump
      break;
    
    case 2:
      //Close
      break;

    case 3:
      //Mid range
      break;

    case 4:
      //long range
      break;

    case 5:
      //Very close range
      break;
  
    default:
      break;
  }
}

public static void stopPitch() {
  turretPitch.set(ControlMode.PercentOutput, 0);
}

public static void startShooterMotors() {
  shooterWheel1.set(ControlMode.PercentOutput, -0.85);
  shooterWheel2.set(ControlMode.PercentOutput, -0.84);
}

public static void stopShooterMotors() {
  shooterWheel1.set(ControlMode.PercentOutput, 0);
  shooterWheel2.set(ControlMode.PercentOutput, 0);
}


}
