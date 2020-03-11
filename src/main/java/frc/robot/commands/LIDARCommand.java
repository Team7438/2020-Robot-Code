package frc.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.LIDARCommand;

/**
 *
 */
public class LIDARCommand extends Command{

    public LIDARCommand() {
        // Use requires() here to declare subsystem dependencies
    	//requires(Robot.lidarSubsystem);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	try {
    	//Robot.lidarSubsystem.initLIDAR(new DigitalInput(RobotMap.LIDAR_PORT));
    	}catch(Exception e) {
    		
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//SmartDashboard.putNumber("LIDAR Distance PWM",Robot.lidarSubsystem.getDistanceIn(true));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}