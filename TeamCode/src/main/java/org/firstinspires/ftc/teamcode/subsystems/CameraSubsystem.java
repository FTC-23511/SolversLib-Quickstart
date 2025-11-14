package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraSubsystem extends SubsystemBase {
    private static final boolean USE_WEBCAM = true;
    private static final List<Integer> DESIRED_TAG_ID = Arrays.asList(21, 22, 23); // Tags we should detect
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor myAprilTagProcessor; // Used for managing the AprilTag detection process.
    //private AprilTagDetection myAprilTagDetection = null; // Used to hold the data for a detected AprilTag
    public CameraSubsystem(final HardwareMap hardwareMap) {
        boolean targetFound = false; // Set to true when an AprilTag target is detected

        // Initialize the Apriltag Detection process
        initAprilTag(hardwareMap);

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        targetFound = false;
        //myAprilTagDetection = null;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(final HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        myAprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        myAprilTagProcessor.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(myAprilTagProcessor)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(myAprilTagProcessor)
                    .build();
        }
    }

    public int detectMotif() {
        List<AprilTagDetection> myAprilTagDetections;
        double area;

        myAprilTagDetections = myAprilTagProcessor.getDetections();
        if (myAprilTagDetections.isEmpty()) {
            return -1;
        }
        AprilTagDetection max = myAprilTagDetections.get(0);
        double max_area = 0.5*Math.abs(
                        (max.corners[0].x * max.corners[1].y)
                        + (max.corners[1].x * max.corners[2].y)
                        + (max.corners[2].x * max.corners[3].y)
                        + (max.corners[3].x * max.corners[0].y)
                        - (max.corners[1].x * max.corners[0].y)
                        - (max.corners[2].x * max.corners[1].y)
                        - (max.corners[3].x * max.corners[2].y)
                        - (max.corners[0].x * max.corners[3].y)
        );
        // Step through the list of detected tags and look for a matching tag
        for (AprilTagDetection detection : myAprilTagDetections) {
            // Look to see if we have size info on this tag.
            if (DESIRED_TAG_ID.contains(detection.id)) {
                area = 0.5*Math.abs(
                        (detection.corners[0].x * detection.corners[1].y)
                        + (detection.corners[1].x * detection.corners[2].y)
                        + (detection.corners[2].x * detection.corners[3].y)
                        + (detection.corners[3].x * detection.corners[0].y)
                        - (detection.corners[1].x * detection.corners[0].y)
                        - (detection.corners[2].x * detection.corners[1].y)
                        - (detection.corners[3].x * detection.corners[2].y)
                        - (detection.corners[0].x * detection.corners[3].y)
                );
                if (area > max_area) {
                    max_area = area;
                    max = detection;
                }
            }
        }
        // check to see if detected tag id matches the available motifs
        return max.id;
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void  setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                //sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            //sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            //sleep(20);
        }
    }

