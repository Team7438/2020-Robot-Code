package frc.robot.jlVision;
import frc.robot.jlVision.AlignVisionBall;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.commands.TurretSearchCmd;
import frc.robot.subsystems.TurretSub;

public class AutoRunTarget extends Command {
    private AlignVisionTarget vision = AlignVisionTarget.getInstance();
    private Boolean rotatingLeft = false;

    public AutoRunTarget() {
    }

    public void search() {
        if (rotatingLeft) {
        TurretSub.setPower(0.7);
        } else if (!rotatingLeft) {
        TurretSub.setPower(-0.7);
        } else {
        TurretSub.setPower(0);
        }
        if (SmartDashboard.getNumber("TurretDistance", 0) <= -100) {
        rotatingLeft = false;
        } else if (SmartDashboard.getNumber("TurretDistance", 0) >= 370) {
        rotatingLeft = true;
        }
    }

    @Override
    protected void initialize() {

    }
    @Override
    protected void execute() {
        //System.out.println(AlignVisionTarget.AugmentedDriverInterface());
        if (SmartDashboard.getBoolean("isValid", false)) {
            TurretSub.setPower(AlignVisionTarget.AugmentedDriverInterface());
        } else {
            search();
        }
    }


    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}