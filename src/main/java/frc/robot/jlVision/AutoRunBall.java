package frc.robot.jlVision;
import frc.robot.jlVision.AlignVisionBall;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.subsystems.DriveSub;

public class AutoRunBall extends Command {
    private AlignVisionBall vision = AlignVisionBall.getInstance();

    public AutoRunBall() {
        requires(Robot.driveSub);
    }

    @Override
    protected void initialize() {

    }
    @Override
    protected void execute() {
        System.out.println("Ball: " + AlignVisionBall.AugmentedDriverInterface());
        DriveSub.betterArcadeDrive(AlignVisionBall.AugmentedDriverInterfaceForward(), AlignVisionBall.AugmentedDriverInterface());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}