package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.List;

@TeleOp(name = "Limelight Functions Test")
public class LimelightFunctionTest extends CommandOpMode {
    private ElapsedTime timer;
    private LimelightSubsystem limelight;
    @Override
    public void initialize() {
        limelight = new LimelightSubsystem(hardwareMap);
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void run() {
        LLResult detections = limelight.getResult();
        List<LLResultTypes.ColorResult> colorTargets = detections.getColorResults();
        List<LLResultTypes.FiducialResult> fiducials = detections.getFiducialResults();

        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.addData("---------subsystem functions---------",null);
        telemetry.addData("detected motif: ", limelight.detectMotif(detections));
        telemetry.addData("goal april tag: ", limelight.findAprilTag(detections));
        telemetry.addData("horizontal distance from goal: ", limelight.detectGoalXDistance(detections));
        telemetry.addData("robot position: ", limelight.detectRobotPosition(detections));
        telemetry.addData("----------random limelight functions--------",null);
        telemetry.addData("target's x: ", detections.getTx());
        telemetry.addData("target's y: ", detections.getTy());
        telemetry.addData("target's area: ", detections.getTa());

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); //ID number of fiducial
            double x = fiducial.getTargetXDegrees(); //Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); //Where it is (up-down)
            telemetry.addData("detected fiducial id: ", id);
            telemetry.addData("where fiducial is (left-right): ", x);
            telemetry.addData("where fiducial is (up-down): ", y);
            telemetry.addData("------------------", null);
            telemetry.addData("robot 3d pose relative to april tag coordinate system ", fiducial.getRobotPoseTargetSpace());
            telemetry.addData("camera 3d pose relative to the april tag ", fiducial.getCameraPoseTargetSpace());
            telemetry.addData("robot 3d pose in the field coordinate system based on this tag alone ", fiducial.getRobotPoseFieldSpace());
            telemetry.addData("april tag pose in the camera's coordinate system ", fiducial.getTargetPoseCameraSpace());
            telemetry.addData("april tag pose in the robot's coordinate system ", fiducial.getTargetPoseRobotSpace());
        }

        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            double x = colorTarget.getTargetXDegrees(); //Where it is (left-right)
            double y = colorTarget.getTargetYDegrees(); //Where it is (up-down)
            double area = colorTarget.getTargetArea();
            telemetry.addData("where color target is (left-right): ", x);
            telemetry.addData("where color target is (up-down): ", y);
            telemetry.addData("% of image the color target takes up: ", area);
        }

        long staleness = detections.getStaleness();
        if (staleness < 100) { // Less than 100 milliseconds old
            telemetry.addData("Data", "Good");
        } else {
            telemetry.addData("Data", "Old (" + staleness + " ms)");
        }
        timer.reset();
        telemetry.update();
    }
}
