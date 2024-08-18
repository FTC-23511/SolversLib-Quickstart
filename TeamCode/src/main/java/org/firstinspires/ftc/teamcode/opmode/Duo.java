package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;


import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Duo extends OpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    ElapsedTime buttonTimer = new ElapsedTime();

    Deposit deposit = new Deposit(hardwareMap);

    @Override
    public void init() {
        deposit.wrist.setPosition(1.0);

        buttonTimer.reset();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        if (gamepad1.left_bumper && buttonTimer.milliseconds() >= 200) {
            runningActions.add(new ParallelAction(
                    new InstantAction(deposit::moveWristLeft),
                    new InstantAction(buttonTimer::reset)
            ));
        } else if (gamepad1.right_bumper && buttonTimer.milliseconds() >= 200) {
            runningActions.add(new ParallelAction(
                    new InstantAction(deposit::moveWristRight),
                    new InstantAction(buttonTimer::reset)
            ));
        }

        if (gamepad1.left_stick_button && checkButton(currentGamepad1, "left stick button")) {
            runningActions.add(new ParallelAction(
                    new InstantAction(deposit::toggleLeftClaw),
                    new InstantAction(buttonTimer::reset)
            ));
        } else if (gamepad1.right_stick_button && checkButton(currentGamepad1, "right stick button")) {
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

        telemetry.addData("wrist", round(deposit.wrist.getPosition(), 2));
        telemetry.addData("leftClaw", round(deposit.leftClaw.getPosition(), 2));
        telemetry.addData("rightClaw", round(deposit.rightClaw.getPosition(), 2));
        telemetry.update();
    }
}