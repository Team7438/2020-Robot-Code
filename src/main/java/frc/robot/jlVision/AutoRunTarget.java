package frc.robot.jlVision;
import frc.robot.jlVision.AlignVisionBall;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.TurretSub;

public class AutoRunTarget extends Command {
    private AlignVisionTarget vision = AlignVisionTarget.getInstance();

    public AutoRunTarget() {
        requires(Robot.driveSub);
    }

    @Override
    protected void initialize() {

    }
    @Override
    protected void execute() {
        //System.out.println(AlignVisionTarget.AugmentedDriverInterface());
        TurretSub.setPower(AlignVisionTarget.AugmentedDriverInterface());
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}