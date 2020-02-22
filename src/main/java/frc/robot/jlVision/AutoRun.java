package frc.robot.jlVision;
import frc.robot.jlVision.AlignVision;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.subsystems.DriveSub;

public class AutoRun extends Command {
    private AlignVision vision = AlignVision.getInstance();

    public AutoRun() {
        requires(Robot.driveSub);
    }

    @Override
    protected void initialize() {

    }
    @Override
    protected void execute() {
        System.out.println(AlignVision.AugmentedDriverInterface());
        DriveSub.betterArcadeDrive(AlignVision.AugmentedDriverInterfaceForward(), AlignVision.AugmentedDriverInterface());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }