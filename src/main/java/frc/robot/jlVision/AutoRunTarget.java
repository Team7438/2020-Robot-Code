package frc.robot.jlVision;
import frc.robot.jlVision.AlignVisionBall;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.commands.TurretSearchCmd;
import frc.robot.subsystems.TurretSub;
import edu.wpi.first.wpilibj.Relay;

public class AutoRunTarget extends Command {
    private AlignVisionTarget vision = AlignVisionTarget.getInstance();
    private Boolean rotatingLeft = false;
    private Relay HPLight = new Relay(0);
    private Double gyroTargetAngle = 0.0;
    private Boolean resetGyroTracker = true;
    private Boolean gyroTrackReady = false;
    private static Boolean gyroTrackOn = true;
    private Double gyroTargetDif = 0.0;

    public AutoRunTarget() {
    }

    public static void turnGyroSearchOn() {
        gyroTrackOn = true;
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

    public void fullSearch() {
        if (rotatingLeft) {
            TurretSub.setPower(0.7);
            } else if (!rotatingLeft) {
            TurretSub.setPower(-0.7);
            } else {
            TurretSub.setPower(0);
            }
            if (SmartDashboard.getNumber("TurretDistance", 0) <= -640) {
            rotatingLeft = false;
            } else if (SmartDashboard.getNumber("TurretDistance", 0) >= 1007) {
            rotatingLeft = true;
            }
    }

    @Override
    protected void initialize() {
        HPLight.setDirection(Relay.Direction.kForward);
        HPLight.set(Relay.Value.kOn);
        resetGyroTracker = true;
    }
    @Override
    protected void execute() {

        if (resetGyroTracker) {
            if (SmartDashboard.getBoolean("isValid", false) && SmartDashboard.getNumber("targetYaw", 100) <= 1 && SmartDashboard.getNumber("targetYaw", 100) >= -1) {
                System.out.println("2222222");
                gyroTargetAngle = SmartDashboard.getNumber("Angle", 0);
                HPLight.set(Relay.Value.kOff);
                TurretSub.stopRotate();
                resetGyroTracker = false;
                gyroTrackReady = true;
            } else {
                System.out.println("1111111");
                if (HPLight.getDescription() == "kOff") {
                    HPLight.set(Relay.Value.kOn);
                }
                fullSearch();
            }
        }

        if (gyroTrackReady && gyroTrackOn) {
            gyroTargetDif = gyroTargetAngle - SmartDashboard.getNumber("Angle", 0);
            if (gyroTargetAngle < 0) {
                AlignVisionTarget.gotoEncoderValue((121.04327885219 + (7.7142950207592 * gyroTargetDif)));
            } else {
                AlignVisionTarget.gotoEncoderValue((90.795400766857 - (7.4326934685651 * gyroTargetDif)));
            }
        } else if (!resetGyroTracker) {
            HPLight.set(Relay.Value.kOn);
            if (SmartDashboard.getBoolean("isValid", false)) {
                TurretSub.setPower(AlignVisionTarget.AugmentedDriverInterface());
            } else {
                search();
            }
            if (SmartDashboard.getNumber("targetYaw", 100) <= 1) {
                gyroTargetAngle = SmartDashboard.getNumber("Angle", 0);
            }
        }

    }

    @Override
    protected void end() {
        HPLight.set(Relay.Value.kOff);
        System.out.println(HPLight.get());
    }


    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }

}