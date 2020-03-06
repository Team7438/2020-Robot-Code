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

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MultiplexedColorSensor;
import frc.robot.Robot;
import frc.robot.subsystems.TurretSub;
import frc.robot.subsystems.BallManagementSub;
import frc.robot.subsystems.ConveyerSub;
import edu.wpi.first.wpilibj.I2C;

public class BallCountCmd extends Command {

  //Sensors
  MultiplexedColorSensor colorSensor0 = new MultiplexedColorSensor(I2C.Port.kMXP, 0);
  MultiplexedColorSensor colorSensor1 = new MultiplexedColorSensor(I2C.Port.kMXP, 1);

  //Sensor ball detection
  private Boolean sensor1BallDetected;
  private Boolean sensor2BallDetected;
  private Boolean sensor3BallDetected;
  private Boolean sensor4BallDetected;
  private Boolean sensor5BallDetected;
 
  //Sensor raw values
  private double sensor1RawValue = 0.0;
  private double sensor2RawValue = 0.0;
  private double sensor3RawValue = 0.0;
  private double sensor4RawValue = 0.0;
  private double sensor5RawValue = 0.0;

  //Ball detection threshold
  private double sensorUpperThreshold = 500.00;
  private double sensorLowerThreshold = 180.00;

  private Boolean runConveyer = false;
  public Integer ballCount = 0;

  public BallCountCmd() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    sensor1RawValue = colorSensor0.getProximity();
    sensor2RawValue = colorSensor1.getProximity();
    


    //Update sensorBallDetected booleans
    if (sensor1RawValue >= sensorUpperThreshold) {
      sensor1BallDetected = true;
    } else if (sensor1RawValue <= sensorLowerThreshold) {
      sensor1BallDetected = false;
    }

    if (sensor2RawValue >= sensorUpperThreshold) {
      sensor2BallDetected = true;
    } else if (sensor2RawValue <= sensorLowerThreshold) {
      sensor2BallDetected = false;
    }

    if (sensor3RawValue >= sensorUpperThreshold) {
      sensor3BallDetected = true;
    } else if (sensor3RawValue <= sensorLowerThreshold) {
      sensor3BallDetected = false;
    }

    if (sensor4RawValue >= sensorUpperThreshold) {
      sensor4BallDetected = true;
    } else if (sensor4RawValue <= sensorLowerThreshold) {
      sensor4BallDetected = false;
    }

    if (sensor5RawValue >= sensorUpperThreshold) {
      sensor5BallDetected = true;
    } else if (sensor5RawValue <= sensorLowerThreshold) {
      sensor5BallDetected = false;
    }

    //If ball is detected in intake
    if (!sensor1BallDetected) {
      if (ballCount <= 3) {
        runConveyer = true;
      } else if (ballCount == 4) {
        //Load into preload chamber
        //Run until sensor 4 detects ball
      } else if (ballCount == 5) {
        //Load into chamber
        //Then preload protocol
      }

    if (runConveyer) {
      ConveyerSub.Rollin();
    } else {
      ConveyerSub.RollStop();
    }

    //When to stop intake
    if (ballCount == 0) {
      if (sensor2BallDetected) {
        runConveyer = false;
      }
    } else if (ballCount == 1) {
        if (sensor3BallDetected) {
          runConveyer = false;
        }
    } else if (ballCount == 2) {
      if (sensor4BallDetected) {
        runConveyer = false;
      }
  } else if (ballCount == 3) {
    //Preload
  } else if (ballCount == 4) {
    //Load
  }

  } 

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    
  }
}