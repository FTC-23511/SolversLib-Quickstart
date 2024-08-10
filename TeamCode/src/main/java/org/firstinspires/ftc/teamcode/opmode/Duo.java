package org.firstinspires.ftc.teamcode.opmode;

import static android.text.TextUtils.substring;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.risingEdge;


import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Duo extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    risingEdge risingEdgeDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Deposit deposit = new Deposit(hardwareMap);

        deposit.wrist.setPosition(1.0);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.left_bumper && checkButton(gamepad1, "left_bumper")) {
                runningActions.add(new SequentialAction(
                    new InstantAction(deposit::moveWristLeft)
                ));

                sleep(200);
            }

            else if (gamepad2.right_bumper && checkButton(gamepad2, "right_bumper")) {
                runningActions.add(new SequentialAction(
                    new InstantAction(deposit::moveWristRight)
                ));

                sleep(200);
            }

            // Updates running actions:
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            String buttons = String.valueOf(gamepad1).substring(75).substring(1);

            telemetry.addData("wristPos", deposit.wrist.getPosition());
            telemetry.addData("gamepad1", checkButton(gamepad1, "triangle"));

            telemetry.addData("gamepad1", buttons.contains("triangle"));
            telemetry.update();
        }
    }

    public boolean checkButton(Gamepad gamepad, String button) {
        String buttons = String.valueOf(gamepad).substring(75).substring(1);
        return buttons.contains(button);
    }
}