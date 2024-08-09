package org.firstinspires.ftc.teamcode.subsystem.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystem.vision.AprilTagsDetection.getCenterStageTagLibrary;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a class to locate the robot position by using the April Tag detections
 */
public class AprilTagDetector {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    //assigns variables to be used in the entire program. not assigned to functions
    public VisionPortal visionPortal;

    public VectorF finalVector;

    public AprilTagLibrary aprilTagLibrary;
    public HardwareMap hardwareMap;

    public double myTagPoseX;
    public double myTagPoseY;
    public boolean tagDetected;
    public double horizontalOffset= 5;
    public double verticalOffset = 7;

    /**
     * The constructor
     * @param _hardwareMap map of the hardware devices connected to this robot
     * @param telemetry telemetry logger
     */
    public AprilTagDetector(HardwareMap _hardwareMap, Telemetry telemetry) {
        // Initialize the April detector here
        hardwareMap = _hardwareMap;
        aprilTagLibrary = getCenterStageTagLibrary();
        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    /**
     * Get the current robot location through April tag detections
     *
     * @return the current robot position and yaw (if successful) | null (otherwise)
     */
    public VectorF getLocation() {

        // Wait for the DS start button to be touched.
        telemetryAprilTag();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int numTags = currentDetections.size();
        if (numTags > 0) {
            telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
        } else {
            telemetry.addData("Tag", "----------- none - ----------");
        }


        // Push telemetry to the Driver Station.2
        telemetry.update();
        visionPortal.close();
        return finalVector;
    }

    public void initAprilTag() {

        // Create the AprilTag processor.
        {
            aprilTag = new AprilTagProcessor.Builder()
/*
 *                   .setDrawAxes(false)
 *                   .setDrawCubeProjection(false)
 *                   .setDrawTagOutline(true)
 *                   .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
 *                   .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
 *                   .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
 *
 *                   // == CAMERA CALIBRATION ==
 *                   // If you do not manually specify calibration parameters, the SDK will attempt
 *                   // to load a predefined calibration for your camera.
 *
 *                   .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
 *
 *                   // ... these parameters are fx, fy, cx, cy.
 *                   .setLensIntrinsics(962.201, 956.583, 320.708, 234.159) // for C270 640x480 from calibdb.net
 *                   .setLensIntrinsics(1443.302, 1434.874, 320.708, 234.159) // for C270 640x480 from calibdb.net
 *                   .setLensIntrinsics(850, 850, 320.708, 234.159) // changed calibdb 640x480 fx,fy to match pose data to actual measurement
 *                   .setLensIntrinsics(1409.503, 1409.503, 632.641, 350.491) // for C270 1280x720 from 3dZephyr
 *
 *                   //.setLensIntrinsics(948.306, 948.306, 323.011, 259.050) // for C270 640x480 from 3dZephyr
 *
 *                   //.setLensIntrinsics(850, 850, 320.708, 234.159) // changed calibdb 640x480 fx,fy to match pose data to actual measurement
 */
                    .setLensIntrinsics(945.447,945.447F,625.861,367.646)// cam calibration c920 cam 1, 1280x720
                    .build();
        }

        // visionPortal
        {
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }
            // Choose a camera resolution. Not all cameras support all resolutions.
            builder.setCameraResolution(new Size(1280, 720));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            // builder.setAutoStopLiveView(false);

            // Set and enable the processor.
            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            visionPortal = builder.build();

            // Disable or re-enable the aprilTag processor at any time.
            //visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }
    @SuppressLint("DefaultLocale")
    public VectorF telemetryAprilTag() {
        List<VectorF> poses = new ArrayList<VectorF>();
        int[] tagYaw = {0, 0, 0, 0, 0, 0, 0,};
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        tagDetected = false;
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            int tagID = detection.id;
            if (detection.metadata != null) {
                tagDetected = true;

                myTagPoseX = detection.ftcPose.x;
                myTagPoseY = detection.ftcPose.y;
                double myTagPoseZ = detection.ftcPose.z;

                double myTagPosePitch = detection.ftcPose.pitch;
                double myTagPoseRoll = detection.ftcPose.roll;
                double myTagPoseYaw = detection.ftcPose.yaw;

                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;

                telemetry.addLine(String.format("\n==== (ID %d) %s", tagID, detection.metadata.name));
                //telemetry.addLine(fieldPose.toString());
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", myTagPoseX, myTagPoseY, myTagPoseZ));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", myTagPosePitch, myTagPoseRoll, myTagPoseYaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", range, bearing, detection.ftcPose.elevation));
                poses.add(getRobotLocation(myTagPoseYaw, myTagPoseZ, range, bearing, tagID));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", tagID));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        //avgFieldLocation[counter] = getRobotLocation();
        // Average all the poses you have, getting a singular pose vector which should be more accurate.
        if (tagDetected) {
            double x = 0;
            double y = 0;
            double z = 0;
//            double yaw = 0;
            for (int i = 0; i < poses.size(); i++) {
                x += poses.get(i).get(0);
                y += poses.get(i).get(1);
                /*
                Z (Unused):
                z += Math.abs(poses.get(i).get(2));

                Yaw:
                yaw += poses.get(i).get(3);
                */

            }
            finalVector = new VectorF((float) x / poses.size(), (float) y / poses.size(), (float) 8/*z / poses.size(), (float)yaw/poses.size()*/);

            telemetry.addLine();
            telemetry.addData("Final Field Position", finalVector);
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nk\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        return finalVector;
    }

    // More private functions here
    public VectorF getRobotLocation(double myTagPoseYaw, double myTagPoseZ, double range, double bearing, int tagID) { //to do average vector, change "void" to vectorF
        myTagPoseY += verticalOffset;
        myTagPoseX += horizontalOffset;
        VectorF fieldPose;
        double fieldY = range * Math.sin(Math.toRadians(bearing - myTagPoseYaw));
        double fieldX = range * Math.cos(Math.toRadians(bearing - myTagPoseYaw));
        //double finalYaw = 0-myTagPoseYaw;

        if (tagID <= 6){
            fieldX *= -1;
            fieldY *= -1;
        }

        // TODO: Add Yaw Here
        // expression may be < 0 if i got y reversed 💀
        fieldPose = new VectorF((float) fieldX, (float) fieldY, (float) - myTagPoseZ);
        fieldPose.add(aprilTagLibrary.lookupTag(tagID).fieldPosition);
        telemetry.addData("Corresponding Field Position", fieldPose);
        return fieldPose;
    }
    public boolean isTagDetected(){
        return tagDetected;
    }
}