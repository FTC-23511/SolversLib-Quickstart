package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class clawTester extends LinearOpMode {
    public static double leftClawPos = 0.5;
    public static double rightClawPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();

        deposit.leftClaw.setPosition(leftClawPos);
        deposit.rightClaw.setPosition(rightClawPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (USE_DASHBOARD){
                deposit.leftClaw.setPosition(leftClawPos);
                deposit.rightClaw.setPosition(rightClawPos);
            } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
                leftClawPos += 0.01;
            } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
                leftClawPos -= 0.01;
            } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
                rightClawPos += 0.01;
            } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
                rightClawPos -= 0.01;
            }

            if (gamepad1.square || gamepad1.cross) {
                deposit.leftClaw.setPosition(leftClawPos);
            } else if (gamepad1.triangle || gamepad1.circle) {
                deposit.rightClaw.setPosition(rightClawPos);
            }

            leftClawPos = Math.max(Math.min(leftClawPos, 1), 0);

            rightClawPos = Math.max(Math.min(rightClawPos, 1), 0);

            currentGamepad1.copy(gamepad1);

            telemetry.addData("leftClaw getPosition", round(deposit.leftClaw.getPosition(), 2));
            telemetry.addData("rightClaw getPosition", round(deposit.rightClaw.getPosition(), 2));
            telemetry.addData("leftClawPos", round(leftClawPos, 2));
            telemetry.addData("rightClawPos", round(rightClawPos, 2));
            telemetry.update();
        }
    }
}