/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Cameras to change, ~line 86, 132

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.Autofront;
//import frc.robot.commands.CentreHatch;
import frc.robot.commands.DriveCmd;
//import frc.robot.commands.AutoRight1;
//import frc.robot.commands.AutoRight2;
//import frc.robot.commands.AutoRight3;
//import frc.robot.commands.AutoRightHatch;
//import frc.robot.commands.AutoLeft1;
//import frc.robot.commands.AutoLeft2;
//import frc.robot.commands.AutoLeft3;
//import frc.robot.commands.ResetElevatorEncoderCommand;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LoaderSub;
//import frc.robot.subsystems.ElevatorTilt;
//import frc.robot.subsystems.ElevatorWinch;
//import frc.robot.subsystems.CargoLoader;
//import frc.robot.subsystems.FrontClimber;
//import frc.robot.commands.ElevatorTiltCmd;
//import frc.robot.commands.ElevatorTune;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import frc.robot.subsystems.HatchRelease;
//import frc.robot.chenyxVision.AutoRun;
// Was commented out import frc.robot.subsystems.PIDElevator;
//import frc.robot.chenyxVision.HUD;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.LIDARLite;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

  

  public static final String MecanumDriver = null;
  public static OI m_oi;
  public static DriveSub driveSub = new DriveSub();
  public static Boolean isEncoderConnected = false;
  public static Integer EncoderFrequency = 0;
  public static Double EncoderOutput;
  public static Double EncoderDistance;
  public static LoaderSub loader = new LoaderSub();
  public NetworkTableEntry targetYaw;
  public NetworkTableEntry isDriverMode;
  public NetworkTableEntry isValid;
  // public static CargoLoader cargoLoader = new CargoLoader();
  // public static HatchRelease hatchRelease = new HatchRelease();
  // public static ElevatorTilt elevatorTilt = new ElevatorTilt();
  // public static ElevatorWinch elevatorWinch = new ElevatorWinch();
  // public static FrontClimber frontClimber = new FrontClimber();
  // was commented out public static UsbCamera camera;
  // private static HUD hud = HUD.getInstance();

  // public Command elevatorTuning = new ElevatorTune();
  public Command drivingCmd = new DriveCmd();
  // Encoder
  public final DutyCycleEncoder m_dutyCycleEncoder = new DutyCycleEncoder(4);
  // Lidar
  public final LIDARLite m_distanceSensor = new LIDARLite(I2C.Port.kOnboard);
  // Color Sensor
  //private final I2C.Port i2Cport = I2C.Port.kMXP;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2Cport);

  public static Shooter shooter0 = new Shooter(0, "New Camera", "http://raspberrypi.local:1181/?action=stream");
  // public static PIDElevator pIDElevatorWinch = new PIDElevator();
  // public static DoubleSolenoid hatchPusher = new
  // DoubleSolenoid(RobotMap.hatchSole1, RobotMap.hatchSole2);

  // public static MecanumDriver mecanumDriver = new MecanumDriver();

  public AHRS ahrs;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

// 2m distance sensor
  private Rev2mDistanceSensor distSens;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

    //2m distance sensor
    distSens = new Rev2mDistanceSensor(Port.kMXP);
    distSens.setAutomaticMode(true);
    distSens.setRangeProfile(RangeProfile.kHighSpeed);
    
    

    m_oi = new OI();

    m_dutyCycleEncoder.setDistancePerRotation(360);
    m_dutyCycleEncoder.reset();
    m_distanceSensor.startMeasuring();

    NetworkTable MicrosoftCam = NetworkTableInstance.getDefault().getTable("chameleon-vision");
    
    // m_chooser.addOption("Center", new Autofront());
    // m_chooser.addOption("Left1", new AutoLeft1());
    // m_chooser.addOption("Left2", new AutoLeft2());
    // m_chooser.addOption("Left3", new AutoLeft3());
    // m_chooser.addOption("centerhatch", new CentreHatch());
    // m_chooser.addOption("Right1", new AutoRight1());
    // m_chooser.addOption("Right2", new AutoRight2());
    // m_chooser.addOption("Right3", new AutoRight3());
    // m_chooser.addOption("RightHatch1", new AutoRightHatch());
    SmartDashboard.putData("Auto mode", m_chooser);
  
    driveSub.encoderReset();
    driveSub.encoderInit();
    driveSub.gyroReset();
    driveSub.gryoInit();

     // Gets the default instance of NetworkTables
     NetworkTableInstance table = NetworkTableInstance.getDefault();

     // Gets the MyCamName table under the chamelon-vision table
     // MyCamName will vary depending on the name of your camera
     NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("New Camera");

     // Gets the yaw to the target from the cameraTable
     targetYaw = cameraTable.getEntry("targetYaw");

     // Gets the driveMode boolean from the cameraTable
     isDriverMode = cameraTable.getEntry("driver_mode");

     // Gets the isValid boolean from the cameraTable

     isValid = cameraTable.getEntry("isValid");
  


    System.out.printf("robotInit");

    //try {
		// 	/***********************************************************************
		// 	 * navX-MXP:
		// 	 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
		// 	 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		// 	 * 
		// 	 * navX-Micro:
		// 	 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
		// 	 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
		// 	 * 
		// 	 * Multiple navX-model devices on a single robot are supported.
		// 	 ************************************************************************/
    //        ahrs = new AHRS(SPI.Port.kMXP);
    //     } catch (RuntimeException ex ) {
    //         DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    //     }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();
    driveSub.encoderUpdate();
    driveSub.gyroUpdate();
    isEncoderConnected = m_dutyCycleEncoder.isConnected();
    EncoderFrequency = m_dutyCycleEncoder.getFrequency();
    EncoderOutput = m_dutyCycleEncoder.get();
    EncoderDistance = m_dutyCycleEncoder.getDistance();


    SmartDashboard.putBoolean("Connected", isEncoderConnected);
    SmartDashboard.putNumber("TurretDistance", EncoderDistance);
    SmartDashboard.putNumber("Lidar Distance", m_distanceSensor.getDistance());
    SmartDashboard.putNumber("targetYaw", targetYaw.getDouble(0.0));
    SmartDashboard.putBoolean("isValid", isValid.getBoolean(false));
    
    //Color detectedColor = m_colorSensor.getColor(); 

    //2m distance sensor
    SmartDashboard.putNumber("Range2M", distSens.getRange());



    /**
     * The sensor returns a raw IR value of the infrared light detected.
     */
    //double IR = m_colorSensor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);

    /**
     * Color Sensor Proximity
     */
    //int proximity = m_colorSensor.getProximity();

    //SmartDashboard.putNumber("Proximity", proximity);

    //Robot.elevatorWinch.eleEncoderUpdate();
    //Robot.elevatorWinch.updateElevatorStatus();
   }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. 
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */

  @Override
  public void autonomousInit() {
    
    m_autonomousCommand = m_chooser.getSelected();

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      //Robot.elevatorWinch.ElevatorEncoderReset();
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (m_oi.joystickZero.getRawButton(9)) {
      m_autonomousCommand.cancel();
      //elevatorTuning.start();
      drivingCmd.start();
    }
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //SmartDashboard.putData("Reset Winch Encoder", new ResetElevatorEncoderCommand());
    //SmartDashboard.putData("ElevatorTilt: Forward", new ElevatorTiltCmd(Value.kForward));
    //SmartDashboard.putData("ElevatorTilt: Back", new ElevatorTiltCmd(Value.kReverse));
    driveSub.gyroReset();
    driveSub.encoderReset();

    //if (!elevatorTuning.isRunning()) {
      // Starts elevator control using the POV control on the joystick
    //  elevatorTuning.start(); 
    //}
    
    //if (m_autonomousCommand != null) {
    //  m_autonomousCommand.cancel();
    //}
  }
// if (!win) {
//   win();
// }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    driveSub.gyroUpdate();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
