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
  public static Joystick blackButtons = new Joystick(4);

  public Boolean isForward = true;
  public static Double tempPower = 0.0;
  public static Double power = 0.0;
  public static Double lastEncoder = 0.0;
  public static Double currentEncoder = 0.0;
  public static Double powerToReturn = 0.0;

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

  public static Double getPowerToMove(Integer speed) {
    // Returns change in power
    currentEncoder = turret_dutyCycleEncoder.getDistance();

    // Speed 1: slow
    // Speed 2: medium
    // Speed 3: fast
    // Speed 4: flippin fast
    if (speed == 1) {
      return 0.0;
    } else if (speed == 2) {
      // 0.04 EU per LU
      //System.out.println((currentEncoder - lastEncoder));
      if ((Math.abs(currentEncoder - lastEncoder)) > 0.07) {
        // decrease power
        lastEncoder = turret_dutyCycleEncoder.getDistance();
        powerToReturn = (-(Math.abs(currentEncoder - lastEncoder)) / 0.1);
        return powerToReturn;
      } else if (((Math.abs(currentEncoder - lastEncoder)) < 0.05)) {
        // increase power
        lastEncoder = turret_dutyCycleEncoder.getDistance();
        powerToReturn = (0.06-(Math.abs(currentEncoder - lastEncoder)));
        //System.out.println(powerToReturn);
        return powerToReturn;
      } else {
        lastEncoder = turret_dutyCycleEncoder.getDistance();
        return 0.0;
      }
    } else {
      return 0.0;
    }
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

  public static void pitchUp() {
    tempPower = getPowerToMove(2);
    power += tempPower;
    if (power > 0.75) {
      power = 0.75;
    }

    if (power < 0.05) {
      power = 0.05;
    }
  turretPitch.set(ControlMode.PercentOutput, -power);
}

public static void pitchDown() {
  tempPower = getPowerToMove(2);
    power += tempPower;
    if (power > 0.15) {
      power = 0.15;
    }

    if (power < 0.05) {
      power = 0.05;
    }

  turretPitch.set(ControlMode.PercentOutput, power);
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
