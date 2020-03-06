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

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */

public class BallManagementSub extends Subsystem {

  public static Solenoid shooterSole = new Solenoid(11, RobotMap.shooterSole2);
  public static VictorSPX turretPitch = new VictorSPX(RobotMap.turretPitch);

  public static TalonSRX shooterWheel1 = new TalonSRX(RobotMap.shooterWheel1);
  public static TalonSRX shooterWheel2 = new TalonSRX(RobotMap.shooterWheel2);

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
  turretPitch.set(ControlMode.PercentOutput, 0.2);
}

public static void pitchDown() {
  turretPitch.set(ControlMode.PercentOutput, -0.2);
}

public static void stopPitch() {
  turretPitch.set(ControlMode.PercentOutput, 0);
}

public static void startShooterMotors() {
  shooterWheel1.set(ControlMode.PercentOutput, -0.85);
  shooterWheel2.set(ControlMode.PercentOutput, -0.85);
}

public static void stopShooterMotors() {
  shooterWheel1.set(ControlMode.PercentOutput, 0);
  shooterWheel2.set(ControlMode.PercentOutput, 0);
}


}
