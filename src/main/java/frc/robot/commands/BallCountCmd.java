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

import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.I2C;

public class BallCountCmd extends Command {

  //Calibration Mode
  private Boolean testingMode = false;

  //Sensors
  MultiplexedColorSensor colorSensor0 = new MultiplexedColorSensor(I2C.Port.kOnboard, 0);
  MultiplexedColorSensor colorSensor1 = new MultiplexedColorSensor(I2C.Port.kOnboard, 1);
  MultiplexedColorSensor colorSensor2 = new MultiplexedColorSensor(I2C.Port.kOnboard, 2);
  MultiplexedColorSensor colorSensor3 = new MultiplexedColorSensor(I2C.Port.kOnboard, 3);
  MultiplexedColorSensor colorSensor4 = new MultiplexedColorSensor(I2C.Port.kOnboard, 4);


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

  private Boolean runConveyer = false;
  public Integer ballCount = 0;
  public Integer tempBallCount = 0;
  public Boolean backABit = false;
  public Integer backABitTimer = 0;

  private Boolean ballToAdd = false;

  public BallCountCmd() {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    ballCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    sensor1RawValue = colorSensor0.getProximity();
    sensor2RawValue = colorSensor1.getProximity();
    sensor3RawValue = colorSensor2.getProximity();
    sensor4RawValue = colorSensor3.getProximity();
    sensor5RawValue = colorSensor4.getProximity();

    SmartDashboard.putNumber("Sensor11", sensor1RawValue);
    SmartDashboard.putNumber("Sensor22", sensor2RawValue);
    SmartDashboard.putNumber("Sensor33", sensor3RawValue);
    SmartDashboard.putNumber("Sensor44", sensor4RawValue);
    SmartDashboard.putNumber("Sensor55", sensor5RawValue);
    


    if (!testingMode) {
      //Update sensorBallDetected booleans
      if (sensor1RawValue >= 340) {
        sensor1BallDetected = true;
      } else if (sensor1RawValue <= 200) {
        sensor1BallDetected = false;
      }

      if (sensor2RawValue >= 400) {
        sensor2BallDetected = true;
      } else if (sensor2RawValue <= 125 ) {
        sensor2BallDetected = false;
      }

      if (sensor3RawValue >= 400) {
        sensor3BallDetected = true;
      } else if (sensor3RawValue <= 150) {
        sensor3BallDetected = false;
      }

      if (sensor4RawValue >= 630) {
        sensor4BallDetected = true;
      } else if (sensor4RawValue <= 200) {
        sensor4BallDetected = false;
      }

      if (sensor5RawValue >= 700) {
        sensor5BallDetected = true;
      } else if (sensor5RawValue <= 300) {
        sensor5BallDetected = false;
      }

      if (backABit) {
        ConveyerSub.Rollout();
        backABitTimer += 1;
        if (backABitTimer >= 5) {
          backABit = false;
          backABitTimer = 0;
          ConveyerSub.RollStop();
        }
      }

      //If ball is detected in intake
      if (sensor1BallDetected) {
        if (ballCount <= 3) {
          
        } else if (ballCount == 3) {
          //Load into preload chamber
          //Run until sensor 4 detects ball
        } else if (ballCount == 4) {
          //Load into chamber
          //Then preload protocol
        } else {
          //Don't let the ball in
        }

        if (!ballToAdd) {
          tempBallCount += 1;
          ballToAdd = true;
        }

      }

      System.out.println(ballCount);

      if (runConveyer) {
        ConveyerSub.Rollin();
      } else {
        ConveyerSub.RollStop();
        ballToAdd = false;
      }

      //When to stop conveyer
      if (ballCount == 0) {
        try {
          if (sensor2BallDetected) {
            runConveyer = false;
            ballCount += tempBallCount;
            tempBallCount = 0;
          }
        } catch (NullPointerException e) {
        }
      } else if (ballCount == 1) {
          try {
            if (sensor3BallDetected) {
              runConveyer = false;
              ballCount += tempBallCount;
              tempBallCount = 0;
            }
          } catch (NullPointerException e) {
          }
      } else if (ballCount == 2) {
        try {
          if (sensor4BallDetected) {
            runConveyer = false;
            ballCount += tempBallCount;
            tempBallCount = 0;
          }
        } catch (NullPointerException e) {
        }
    } else if (ballCount == 3) {
      try {
        if (sensor5BallDetected) {
          runConveyer = false;
          backABit = true;
          ballCount += tempBallCount;
          tempBallCount = 0;
        }
      } catch (NullPointerException e) {
      }
    }
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