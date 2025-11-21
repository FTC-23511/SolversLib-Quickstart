package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraSubsystem extends SubsystemBase {
    private static final List<Integer> MOTIF_TAG_IDS = Arrays.asList(21, 22, 23); // Tags we should detect for motif
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24); // Tags we should detect for goal
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTagProcessor; // Used for managing the AprilTag detection process.
    private void setManualExposure(int exposureMS, int gain) {
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
    public CameraSubsystem() {

    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }
    public void setAprilTagProcessor(AprilTagProcessor aprilTagProcessor) {
        this.aprilTagProcessor = aprilTagProcessor;
    }

    public void setVisionPortal(VisionPortal visionPortal) {
        this.visionPortal = visionPortal;
    }

    /**
     * @return performs hardware call on camera to return list of detections
     * */
    public List<AprilTagDetection> detectAprilTags() {
        List<AprilTagDetection> myAprilTagDetections;
        myAprilTagDetections = aprilTagProcessor.getDetections();
        return myAprilTagDetections;
    }

    /**
     * @return pass in list of april tag detections (to not use hardware call in method) to detect motif as a Motifs object
     * */
    public Object detectMotifID(List<AprilTagDetection> aprilTag) {
        double area;
        if (aprilTag.isEmpty()) {
            return null;
        }
        AprilTagDetection max = aprilTag.get(0);
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
        for (AprilTagDetection detection : aprilTag) {
            // Look to see if we have size info on this tag.
            if (MOTIF_TAG_IDS.contains(detection.id)) {
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

    /**
     * @return pass in list of april tag detections (to not use hardware call in method) to get camera's direct (point-to-point) distance to the tag center as a double or null if nothing is found
     * */
    public Object detectGoalDistance(List<AprilTagDetection> aprilTag) {
        // check to see if detected tag id matches the available motifs
        for (AprilTagDetection detection : aprilTag) {
            // april tag pose
            if (GOAL_TAG_IDS.contains(detection.id)) {
                return detection.ftcPose.range;
            }
        }
        return null;
    }

    /**
     * @return pass in list of april tag detections (to not use hardware call in method) to  get camera's horizontal distance from tag center as a double or null if nothing is found
     * */
    public Object detectGoalXDistance(List<AprilTagDetection> aprilTag) {
        // check to see if detected tag id matches the available motifs
        for (AprilTagDetection detection : aprilTag) {
            // april tag pose
            if (GOAL_TAG_IDS.contains(detection.id)) {
                return detection.ftcPose.x;
            }
        }
        return null;
    }

    /**
     * @return pass in list of april tag detections (to not use hardware call in method) to get the detected apriltag as an AprilTagDetection or null if none is found
     * */
    public Object findAprilTag(List<AprilTagDetection> aprilTag) {
        // check to see if detected tag id matches the available motifs
        for (AprilTagDetection detection : aprilTag) {
            // april tag pose
            if (GOAL_TAG_IDS.contains(detection.id)) {
                return detection;
            }
        }
        return null;
    }

    /**
     * !Must be ran after apriltagprocessor and vision portal are passed in!
     */
    public void initializeSettings() {
        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        aprilTagProcessor.setDecimation(3);
    }
}

