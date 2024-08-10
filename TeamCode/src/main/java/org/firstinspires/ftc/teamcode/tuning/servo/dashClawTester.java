package org.firstinspires.ftc.teamcode.tuning.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class dashClawTester extends LinearOpMode {

    public static double leftClawPos = 0.5;
    public static double rightClawtPos = 0.5;

    public static boolean moveLeft = false;
    public static boolean moveRight = false;
    public static boolean moveBoth = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        deposit.leftClaw.setPosition(leftClawPos);
        deposit.rightClaw.setPosition(rightClawtPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (moveLeft) {
                deposit.leftClaw.setPosition(leftClawPos);

                moveLeft = false;
            }

            if (moveRight) {
                deposit.rightClaw.setPosition(rightClawtPos);

                moveRight = false;
            }

            if (moveBoth) {
                deposit.leftClaw.setPosition(leftClawPos);
                deposit.rightClaw.setPosition(rightClawtPos);

                moveBoth = false;
            }

            telemetry.addData("leftWrist getPosition", deposit.leftClaw.getPosition());
            telemetry.addData("rightWrist getPosition", deposit.rightClaw.getPosition());

            telemetry.addData("leftWristPos", leftClawPos);
            telemetry.addData("rightWristPos", rightClawtPos);

            telemetry.addData("moveLeft", moveLeft);
            telemetry.addData("moveRight", moveRight);
            telemetry.addData("moveBoth", moveBoth);
            telemetry.update();
        }
    }
}