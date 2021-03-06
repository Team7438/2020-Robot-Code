package frc.robot.jlVision;
import org.opencv.core.Point;
import frc.robot.*;
import frc.robot.subsystems.TurretSub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignVisionTarget {

    private static AlignVisionTarget instance;
    public static AlignVisionTarget getInstance() {
        if(instance == null)
            synchronized(AlignVisionTarget.class){
                if(instance == null)
                    instance = new AlignVisionTarget();
            }
        return instance;
    }


    private static double turnPower = 0.1;

    private static double tempPower = 0.2;
    public static double tempVar;

    public volatile double timestamp=0, lastLockTime=0;
    public volatile boolean isConnected;

    private static double distanceToTravel = 0;

    public void AugmentedDriving() {
        //Filler
    }


    private static double turnRate(double p) {

        tempPower = Math.abs(p / 12);

        if (tempPower < 0.05) {
            return 0;
        }

        if (tempPower < 0.1) {
            tempPower = 0.1;
        } else if (tempPower > 0.6) {
            tempPower = 0.6;
        }

        if (p > 0) {
            return tempPower * -1;
        } else if (p < 0) {
            return tempPower;
        } else {
            return 0;
        }

    }

    public static double AugmentedDriverInterface() {
        tempVar = SmartDashboard.getNumber("targetYaw", 0);

        Point delta = new Point();
        System.out.println(delta.x);
        if(Robot.shooter0.trainOnTraget(delta)) {
        // rotate turret by delta.x degrees relative to current position
        // pitch the shooter by delta.y degrees relative to current position

        }

        //System.out.println("TARGET: " + tempVar);
        turnPower = turnRate(tempVar);
        return turnPower;
    }

    public static void gotoEncoderValue(double EUGo) {
        // Get distance between the EU to go.
        distanceToTravel = SmartDashboard.getNumber("TurretDistance", 132) - EUGo;
        turnPower = Math.abs(distanceToTravel)/20;
        //System.out.println(turnPower);
        if (turnPower > 0.7) {
            turnPower = 0.7;
        }
        if (turnPower < 0.1) {
            turnPower = 0.1;
        }
        if (distanceToTravel < 0) {
            TurretSub.setPower(-turnPower);
        } else {
            TurretSub.setPower(turnPower);
        }
    }

}