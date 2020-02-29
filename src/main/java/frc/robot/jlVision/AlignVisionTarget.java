package frc.robot.jlVision;
import org.opencv.core.Point;
import frc.robot.*;


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
        if(Robot.shooter0.trainOnTraget(delta)) {
        // rotate turret by delta.x degrees relative to current position
        // pitch the shooter by delta.y degrees relative to current position

        }

        //System.out.println("TARGET: " + tempVar);
        turnPower = turnRate(tempVar);
        return turnPower;
    }

}