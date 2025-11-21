package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Camera Detection Test")
public class CameraDetectionTest extends CommandOpMode {
    private CameraSubsystem camera;
    boolean cameraInitialized = false;
    @Override
    public void initialize() {
        camera = new CameraSubsystem();
    }

    @Override
    public void run() {
        if (!cameraInitialized) {
            camera.setAprilTagProcessor(new AprilTagProcessor.Builder()
                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.
                    .build());
            camera.setVisionPortal(new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(camera.getAprilTagProcessor())
                    .build()
            );
            cameraInitialized = true;
        }
        List<AprilTagDetection> detections = camera.detectAprilTags();
        telemetry.addData("detected motif: ", camera.detectMotifID(detections));
        telemetry.addData("goal point-to-point distance: ", camera.detectGoalDistance(detections));
        telemetry.addData("goal horizontal distance: ", camera.detectGoalXDistance(detections));
        telemetry.addData("april tag: ", camera.findAprilTag(detections));
        telemetry.update();
    }
}
