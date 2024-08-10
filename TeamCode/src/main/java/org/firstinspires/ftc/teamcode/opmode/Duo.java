package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.risingEdge;


import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Duo extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        ElapsedTime buttonTimer = new ElapsedTime();

        Deposit deposit = new Deposit(hardwareMap);

        deposit.wrist.setPosition(1.0);

        buttonTimer.reset();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.left_bumper && buttonTimer.milliseconds() >= 200) {
                runningActions.add(new ParallelAction(
                    new InstantAction(deposit::moveWristLeft),
                    new InstantAction(buttonTimer::reset)
                ));
            }

            else if (gamepad1.right_bumper && buttonTimer.milliseconds() >= 200) {
                runningActions.add(new ParallelAction(
                    new InstantAction(deposit::moveWristRight),
                    new InstantAction(buttonTimer::reset)
                ));
            }

            if (gamepad1.left_stick_button && buttonTimer.milliseconds() >= 200) {
                runningActions.add(new ParallelAction(
                        new InstantAction(deposit::toggleLeftClaw),
                        new InstantAction(buttonTimer::reset)
                ));
            }

            if (gamepad1.right_stick_button && buttonTimer.milliseconds() >= 200) {
                runningActions.add(new ParallelAction(
                        new InstantAction(deposit::toggleRightClaw),
                        new InstantAction(buttonTimer::reset)
                ));
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


            telemetry.addData("leftClaw Pos", new BigDecimal((String.valueOf(deposit.leftClaw.getPosition()))).setScale(2, RoundingMode.HALF_UP).doubleValue());
            telemetry.addData("rightClaw Pos", new BigDecimal((String.valueOf(deposit.rightClaw.getPosition()))).setScale(2, RoundingMode.HALF_UP).doubleValue());
            telemetry.update();
        }
    }
}