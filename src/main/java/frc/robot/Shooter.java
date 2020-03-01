package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*

    Initalize:
    import org.opencv.core.Point;
    // for game use:
    Shooter shooter0 = new Shooter(0, "C922 Pro Stream Webcam");
    // for visual mode (for debugging) use:
    Shooter shooter0 = new Shooter(0, "C922 Pro Stream Webcam", "http://raspberrypi.local:1181/?action=stream");

    Call:
    Point delta = new Point();
    if(shooter0.trainOnTraget(delta)) {
      // rotate turret by delta.x degrees relative to current position
      // pitch the shooter by delta.y degrees relative to current position

    }

*/

public class Shooter {

    int camera_id = -1;
    String camera_url = null;
    String camera_name = null;
    CvSink cvSink = null;
    CvSource outputStream = null;
    NetworkTable chameleon_table = null;
    NetworkTable camera_table = null;

    int displayed_frame_h_width = -1; // half-sizes
    int displayed_frame_h_height = -1;
    boolean HD_format = true;

    double FOV_h_hor = -1.0; // 29/36
    double FOV_h_ver = -1.0; // 23/20

    // manual calibration: at distance calibDist
    double calibDist = -1.0; // meters
    double calibLenX = -100;
    double calibLenY = -50;

    int targetLocked = 0;

    boolean driverMode = false; // don't show overlays on top of video--unused (does not populate NT)
    boolean headlessMode = false; // do not get or show any video to preserve bandwidth; only data from NT

    double calibBoundingWidth = -1.0f;

    boolean isValid = false;
    Point shooterDelta = new Point(); // (yaw, pitch)

    double hexgon_width_inches = 39.25f;

    // Constructors

    // headerless constructor (do not get video frames); use for game:
    // Shooter shooter0 = new Shooter(0, "C922 Pro Stream Webcam");
    public Shooter(int camera_id, String camera_name) {

        this.camera_id = camera_id;
        this.camera_name = camera_name;

        headlessMode = true;

        chameleon_table = NetworkTableInstance.getDefault().getTable("chameleon-vision/" + camera_name);

        new Thread(() -> {

            while (!Thread.interrupted()) {
                // get frame sizes
                displayed_frame_h_width = 640 / 2;
                displayed_frame_h_height = 380 / 2;
                annotateFrame(null);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }).start();

    }

    // visual constructor: shows Chameleon video frames and overlays ballistic target:
    // Shooter shooter0 = new Shooter(0, "C922 Pro Stream Webcam", "http://raspberrypi.local:1181/?action=stream");
    public Shooter(int camera_id, String camera_name, String camera_url) {

    this.camera_id = camera_id;
    this.camera_url = camera_url;
    this.camera_name = camera_name;

    chameleon_table = NetworkTableInstance.getDefault().getTable("chameleon-vision/" + camera_name);

    /* get video resolutions from CameraPublisher--failing
    camera_table = NetworkTableInstance.getDefault().getTable("CameraPublisher/C922 Pro Stream Webcam");

    // get video resolution dynamically
    NetworkTableEntry entry = NetworkTableInstance.getDefault().getEntry("CameraPublisher");
    System.out.println("Entry: " + entry.getValue());

    System.out.println("Keys: " + camera_table.getKeys().size());
    for(String aKey : camera_table.getKeys())  System.out.println("Key: " + aKey);
    String mode = camera_table.getEntry("mode").getString("unknown");
    System.out.println("Mode: " + mode);
    */

    /* get video resolutions dynamically from first frame instead
    if(camera_id ==0) {
        displayed_frame_h_width  = 640/2;  // 320/640/960
        displayed_frame_h_height = 480/2;  // 240/360/540
        HD_format = true;
    } else {
        displayed_frame_h_width  = 1024/2;  // 320/640/960
        displayed_frame_h_height = 768/2;  // 240/360/540
        HD_format = false;
    }
    */

    new Thread(() -> {

        /* Access Chameleon HTTP camera */
        HttpCamera chameleonFeed = new HttpCamera("chameleon."+camera_id, camera_url, HttpCameraKind.kMJPGStreamer);
        MjpegServer camera = CameraServer.getInstance().startAutomaticCapture(chameleonFeed);
        cvSink = CameraServer.getInstance().getVideo("chameleon."+camera_id);

        /* another option for HTTP cconnected camera, but does not work
        int junk = CameraServerJNI.createHttpCamera("Cam 1", "http://raspberrypi.local:1181/?action=stream,mjpg:http://10.0.0.225", 0);
        */

        /*  try AxisCamera approach--could not get it to work
        AxisCamera camera = CameraServer.getInstance().addAxisCamera("AxisCam", "http://raspberrypi.local:1181/?action=stream,");
        CvSink cvSink = CameraServer.getInstance().getVideo("AxisCam");
        */

        /* use local USB camera
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("C922 Pro Stream Webcam", 0);
        // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        CvSink cvSink = CameraServer.getInstance().getVideo();
        */

        /* set different screen resolution
        camera.setResolution(2*displayed_frame_h_width, 2*displayed_frame_h_height);
        */

        outputStream = CameraServer.getInstance().putVideo("Shooter."+camera_id, 2*displayed_frame_h_width, 2*displayed_frame_h_height);

        Mat source = new Mat();
        // Mat source2 = new Mat(1024,768, 16);

        while(!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source Mat.  If there is an error notify the output.
            // if (false) {
            if (cvSink.grabFrame(source) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // get frame sizes
            displayed_frame_h_width  = source.cols()/2;
            displayed_frame_h_height = source.rows()/2;
            // System.out.println("video frame sizes: " + displayed_frame_h_width + " " + displayed_frame_h_height);
            annotateFrame(source);
            outputStream.putFrame(source);
        }
    }).start();

    }

    private void annotateFrame(Mat source) {

        // manual calibration
        if (HD_format) { // HD ratio
            // shooter axis offset
            if (camera_id == 0) {  // camera.0 is to the left, camera.1 to the right
                calibLenX = -100;
                calibLenY = -50;
            } else {
                calibLenX = 100;
                calibLenY = -50;
            }

            FOV_h_hor = 36.0;
            FOV_h_ver = 20.0;
        } else { // 0.75 ratio
            // shooter axis offset
            if (camera_id == 0) {
                calibLenX = -100;
                calibLenY = -100;
            } else {
                calibLenX = 100;
                calibLenY = -100;
            }

            FOV_h_hor = 36.0;
            FOV_h_ver = 24.0;
        }

        calibDist = camera_id == 0 ? 1.0f : 1.7f; // meters
        calibBoundingWidth = camera_id == 0 ? 435.0f : 370f; // in pixels

        // source.setTo(new Scalar(0,0,0));  // clear image

        driverMode = chameleon_table.getEntry("driverMode").getBoolean(false);

        double targetPitch = chameleon_table.getEntry("targetPitch").getDouble(0.0);
        double targetYaw   = chameleon_table.getEntry("targetYaw").getDouble(0.0);

        // display target in camera reference frame in cyan
        Point cameraTarget = new Point(0,0);
        cameraTarget.x = displayed_frame_h_width *(1+Math.sin(targetYaw  *Math.PI/180.0f)/Math.sin(FOV_h_hor*Math.PI/180.0f));
        cameraTarget.y = displayed_frame_h_height*(1-Math.sin(targetPitch*Math.PI/180.0f)/Math.sin(FOV_h_ver*Math.PI/180.0f));
        // System.out.println("CameraTarget: " + cameraTarget);

        if( ! headlessMode)
            Imgproc.circle(source, cameraTarget, 20, new Scalar(255,255,0), 5);


        // targetPose[0] is distance to the target in meters as provided by Chameleon
        double[] pose   = chameleon_table.getEntry("targetPose").getDoubleArray(new double[]{ -1.0, -1.0, -1.0 });
        double distance = pose[0];
        boolean targetVisible = true;
        if(pose[0] == 0.0 && pose[1] == 0.0) targetVisible = false;
        if( ! targetVisible) distance = 2.0;
        // System.out.println("TargetDistance: " + distance);
        // distance_horiz is in cm
        double distance_horiz = chameleon_table.getEntry("NT_LOC_HERE/distance_horiz").getDouble(-1.0);  // TODO get from NT


        // shooterSight is the point on the screen where the shooter axis intersects image plane (in pixels)
        // shooter is located to the left and up of camera axis
        // lineraly interpolate the position based on manual calibration: relative to: top edge length in pixels at calibration distance
        Point shooterSight = new Point(0,0);
        shooterSight.x = displayed_frame_h_width  + calibLenX * distance/calibDist;
        shooterSight.y = displayed_frame_h_height + calibLenY * distance/calibDist;

        double shooterYaw   = Math.asin((shooterSight.x/displayed_frame_h_width -1.0f)*Math.sin(FOV_h_hor*Math.PI/180.0f))*180.0f/Math.PI;
        double shooterPitch = Math.asin((1.0f-shooterSight.y/displayed_frame_h_height)*Math.sin(FOV_h_ver*Math.PI/180.0f))*180.0f/Math.PI;

        // draw shooterSight in yellow
        if( ! headlessMode)
            Imgproc.circle(source, shooterSight, 20, new Scalar(0,255,255), 5);

        double velocity = 10.0f;  // TODO get from NT
        double scale = (calibBoundingWidth/hexgon_width_inches) * (distance / calibDist);  // scale from inches to pixels (linear interp)
        Point ballisticTarget = balistic_adjustment(shooterSight, distance, distance_horiz, velocity, shooterPitch, scale);



        // draw the ballistic target (where the ball would hit the image plane)
        if(targetVisible && ! headlessMode)
            Imgproc.circle(source, ballisticTarget, 20, new Scalar(0,0,255), 5);

        double ballisticYaw   = Math.asin((ballisticTarget.x/displayed_frame_h_width -1.0f)*Math.sin(FOV_h_hor*Math.PI/180.0f))*180.0f/Math.PI;
        double ballisticPitch = Math.asin((1.0f-ballisticTarget.y/displayed_frame_h_height)*Math.sin(FOV_h_ver*Math.PI/180.0f))*180.0f/Math.PI;

        SmartDashboard.putNumber("camera." + camera_id + "/ballisticYaw", ballisticYaw);
        SmartDashboard.putNumber("camera." + camera_id + "/ballisticPitch", ballisticPitch);

        // shooterDelta is the difference (in degrees) between the corrent orientation of the shooter
        // and desired orientation to hit the target adjusted for shooter axis offset and ballistic drop
        // when it approaches (0,0) it's time to shoot ...
        shooterDelta.x = targetYaw - ballisticYaw;
        shooterDelta.y = targetPitch - ballisticPitch;
        System.out.println("shooterDelta:" + shooterDelta.toString());

        isValid = chameleon_table.getEntry("isValid").getBoolean(false);

        SmartDashboard.putNumber("camera." + camera_id + "/shooterDeltaYaw", shooterDelta.x);
        SmartDashboard.putNumber("camera." + camera_id + "/shooterDeltaPitch", shooterDelta.y);

        double boundingWidth = chameleon_table.getEntry("targetBoundingWidth").getDouble(0.0);
        double ballRadius = 7.0 / 39.25 * calibBoundingWidth * (boundingWidth/calibBoundingWidth) / 2.0f;
        ballRadius *= 1.2f;
        // System.out.println("ballRadius: " + ballRadius);

        if(targetVisible && ! headlessMode)
            Imgproc.circle(source, ballisticTarget, (int)ballRadius, new Scalar(0,0,255), 2);

        // Target locked indicator
        double error = (cameraTarget.x - ballisticTarget.x)*(cameraTarget.x - ballisticTarget.x) + (cameraTarget.y - ballisticTarget.y)*(cameraTarget.y - ballisticTarget.y);
        // System.out.println("error: " + error);

        Scalar locked = new Scalar(0,255,0);
        Scalar not_locked = new Scalar(0,0,255);
        if(! headlessMode)
            Imgproc.circle(source, new Point(55,55), 50, (error < 4.0*(ballRadius*ballRadius) && targetVisible) ? locked : not_locked, -1);

    }

    // calculate screen position (in pixels) of the projectile impact location
    //
    // target_distance is distance to target, horiz_distance is horizontal distance to target
    // if both positive, use horiz_distance
    // target_distance in meters (from Chameleon)
    // horiz_distance in cm (from range finder)
    // velocity in feet/sec
    private Point balistic_adjustment(Point shooterTarget, double target_distance, double horiz_distance, double velocity, double pitch, double scale) {

        Point ballisticTarget = new Point();
        ballisticTarget.x = shooterTarget.x;
        ballisticTarget.y = shooterTarget.y;

        // ballistic calculation

        // calculate hosrizontal distance to target from target_distance
        double x = target_distance*Math.sin(Math.PI*pitch/180.0f);  // in meters
        // or use horiz_distance directly if provided (but convert from cm to meters)
        if (horiz_distance > 0)
            x = horiz_distance *0.01f; // * 0.0254;  old: inches

        // convert velocity from feet to meters
        velocity *= 0.3048;

        final double g = 9.81f;
        double temp = velocity * Math.cos(Math.PI*pitch/180.0f);
        double drop =  g * x * x / (2*temp*temp);

        // convert result from meters to inches
        drop /= 0.0254;
        drop *= scale;
        drop *= 50.0f;  // TODO: kludge factor
        // System.out.println("drop: " + drop);

        // TODO: override with a constant, for debugging
        ballisticTarget.y += 150;  // harcoded ballistic offset, in pixels

        return ballisticTarget;
    }

    public boolean trainOnTraget(Point delta) { // (yaw, pitch)

        delta.x = shooterDelta.x;
        delta.y = shooterDelta.y;

        return isValid;  // add others error conditions and low-pass HERE
    }

}
