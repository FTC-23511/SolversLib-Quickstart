package org.firstinspires.ftc.teamcode.tuning.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
@TeleOp
public class dashClawTester extends LinearOpMode {

    public static double leftClawPos = 0.5;
    public static double rightClawPos = 0.5;

    public static boolean moveLeft = false;
    public static boolean moveRight = false;
    public static boolean moveBoth = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        deposit.leftClaw.setPosition(leftClawPos);
        deposit.rightClaw.setPosition(rightClawPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (moveLeft) {
                deposit.leftClaw.setPosition(leftClawPos);

                moveLeft = false;
            }

            if (moveRight) {
                deposit.rightClaw.setPosition(rightClawPos);

                moveRight = false;
            }

            if (moveBoth) {
                deposit.leftClaw.setPosition(leftClawPos);
                deposit.rightClaw.setPosition(rightClawPos);

                moveBoth = false;
            }

            telemetry.addData("leftWrist getPosition", deposit.leftClaw.getPosition());
            telemetry.addData("rightWrist getPosition", deposit.rightClaw.getPosition());

            telemetry.addData("leftClaw Pos", new BigDecimal((String.valueOf(deposit.leftClaw.getPosition()))).setScale(2, RoundingMode.HALF_UP));
            telemetry.addData("rightClaw Pos", new BigDecimal((String.valueOf(deposit.rightClaw.getPosition()))).setScale(2, RoundingMode.HALF_UP));

            telemetry.addData("leftWristPos", leftClawPos);
            telemetry.addData("rightWristPos", rightClawPos);

            telemetry.addData("moveLeft", moveLeft);
            telemetry.addData("moveRight", moveRight);
            telemetry.addData("moveBoth", moveBoth);
            telemetry.update();
        }
    }
}