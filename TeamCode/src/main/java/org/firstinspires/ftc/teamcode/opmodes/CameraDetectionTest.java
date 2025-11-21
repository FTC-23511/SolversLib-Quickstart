package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Camera Detection Test")
public class CameraDetectionTest extends CommandOpMode {
    private CameraSubsystem camera;
    @Override
    public void initialize() {
        camera = new CameraSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        camera.initAprilTag(hardwareMap);
        List<AprilTagDetection> detections = camera.detectAprilTags();
        telemetry.addData("detected motif: ", camera.detectMotif(detections));
        telemetry.addData("goal point-to-point distance: ", camera.detectGoalDistance(detections));
        telemetry.addData("goal horizontal distance: ", camera.detectGoalXDistance(detections));
        telemetry.addData("april tag: ", camera.findAprilTag(detections));
        telemetry.update();
    }
}
