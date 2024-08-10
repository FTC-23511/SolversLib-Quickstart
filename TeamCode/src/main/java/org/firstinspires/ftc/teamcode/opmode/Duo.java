package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.risingEdge;


import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Duo extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    risingEdge risingEdgeDetector;

    String buttons = "";

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
            
            if (gamepad1.left_bumper && checkButton(currentGamepad1, "left_bumper")) {
                runningActions.add(new SequentialAction(
                    new InstantAction(deposit::moveWristLeft)
                ));

                sleep(200);
            }

            if (gamepad1.right_bumper && checkButton(currentGamepad1, "right_bumper")) {
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

            telemetry.addData("wristPos", deposit.wrist.getPosition());
            telemetry.update();
        }
    }

    public boolean checkButton(Gamepad gamepad, String button) {
        try {
            String buttons = String.valueOf(gamepad).substring(75).substring(1);
            return !buttons.contains(button);
        }
        catch (Exception ignored){
            return (true);
        }
    }
}