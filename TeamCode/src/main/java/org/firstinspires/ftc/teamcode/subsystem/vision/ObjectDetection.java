package org.firstinspires.ftc.teamcode.subsystem.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;
@Disabled
@Autonomous
public abstract class ObjectDetection extends LinearOpMode {
    private final int resolutionWidth = 1920;
    private final int resolutionHeight = 1080;
    private VisionPortal visionPortal;

    public void runOpmode(){
        initVision();
        int frameCount = 0;
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            visionPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
        }

    }
    private void initVision(){
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(resolutionWidth,resolutionHeight))
                .enableLiveView(true)
                .build();

    }
}