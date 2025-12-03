package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.limelightvision.LLResult;
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
        timer.reset();
        telemetry.update();
    }
}
