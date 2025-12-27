package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LimelightSubsystem extends SubsystemBase {
    Limelight3A limelight;
    private static final List<Integer> MOTIF_TAG_IDS = Arrays.asList(21, 22, 23); // Tags we should detect for motif
    private static final List<Integer> GOAL_TAG_IDS = Arrays.asList(20, 24); // Tags we should detect for goal
    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(90); // This sets how often we ask Limelight for data (90 times per second)
        //Probably our pipelines will look something like this:
        //  0: Apriltag/megatag2
        //  1: Color blob detection
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        limelight.start(); // This tells Limelight to start looking!

        ;
    }

    /**
     * @return performs hardware call on limelight to return a detection
     * */
    public LLResult getResult() {
        return limelight.getLatestResult();
    }
    /**
     * @return pass in list of detections to detect motif id num (21, 22, or 23)
     * */
    public Object detectMotif(LLResult result) {
        //TODO for Claire: Make this method return the biggest motif fiducial! rn it returns the first tag with a motif it can find.
        //fiducial.getTargetArea() might work?? i havent looked into it 
        //https://docs.steelbootrobotics.org/docs/Documents/limelight%20stuff.pdf
        // Obelisk GPP ID = 21
        // Obelisk PGP ID = 22
        // Obelisk PPG ID = 23
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (MOTIF_TAG_IDS.contains(id)) {
                    return id;
                }
            }
        }
        return null;
    }



    /**
     * @param result a detection from the limelight
     * Returns the horizontal distance of the center crosshair to the goal apriltags. Used for camera-only autoaim (not preferrable).
     * @return camera's horizontal distance from each specific tag's center as a double or null if nothing is found
     * */
    public Object detectGoalXDistance(LLResult result) {
        // Red ID = 24
        // Blue ID = 20
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (GOAL_TAG_IDS.contains(id)) {
                    Pose3D botpose_mt2 = result.getBotpose_MT2();
                    if (botpose_mt2 != null) {
                        double x = botpose_mt2.getPosition().x;
                        double y = botpose_mt2.getPosition().y;
                        double z = botpose_mt2.getPosition().z;

                        x = DistanceUnit.INCH.fromMeters(x);
                        y = DistanceUnit.INCH.fromMeters(y);
                        z = DistanceUnit.INCH.fromMeters(z);
                        return x;
                    }
                }
            }
        }
        return null;
    }

    //TODO claire: we might need these in pedro's coordinate system.
    /**
     * @param result limelight detection
     * @return robot's position in FTC coordinate system as a set of coordinate points or null if nothing is found
     * */
    public Object detectRobotPosition(LLResult result) {
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                return new double[]{x,y};
            }
        }
        return null;
    }

    //also claire remember u can use @param to annotate what you need to pass in
    /**
     * @return pass in list of detections to get the detected goal apriltag id or null if none is found
     * */
    public Object findAprilTag(LLResult result) {
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (GOAL_TAG_IDS.contains(id)) {
                    return id;
                }
            }
        }
        return null;
    }
    /**
     * @param result ll result
     * @return The angle that the closest (?) ball is from the camera, in degrees.
     */
    public Object findColorBlobHeading(LLResult result) {
        if (result != null && result.isValid()) {
            return result.getTx(); //Apparently, tx is in degrees already.
        }
        return null;
    }

    /**
     * @param result ll result
     * @return The distance that the closest (?) ball is from the camera, in inches.
     */
    public Object findColorblobDistance(LLResult result) {
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double mountingHeight = 8.45; //The height of the camera lens from the floor (in)
            double mountingAngle = 0.0; //The mounting angle of the camera relative to the horizon (deg).
            double ballHeight = 5.0 / 2.0; //The height of the center of the ball (in)
            double distance = (ballHeight - mountingHeight) / Math.tan(Math.toRadians(ty + mountingAngle));
            return distance;
        }
        return null;
    }
}

