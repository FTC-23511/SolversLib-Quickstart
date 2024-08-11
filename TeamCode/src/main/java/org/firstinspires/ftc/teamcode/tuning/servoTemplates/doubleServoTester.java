package org.firstinspires.ftc.teamcode.tuning.servoTemplates;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.risingEdge;

import java.math.BigDecimal;
import java.math.RoundingMode;

@Config
@TeleOp
public class doubleServoTester extends LinearOpMode {
    public static double leftServoPos = 0.5;
    public static double rightServoPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Subsystem subsystem = new Subsystem(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();

        subsystem.leftServo.setPosition(leftServoPos);
        subsystem.rightServo.setPosition(rightServoPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (USE_DASHBOARD){
                subsystem.leftServo.setPosition(leftServoPos);
                subsystem.rightServo.setPosition(rightServoPos);
            } else if (gamepad1.dpad_up  && risingEdge.checkButton(currentGamepad1, "dpad_up")) {
                leftServoPos += 0.01;
            } else if (gamepad1.dpad_down && risingEdge.checkButton(currentGamepad1, "dpad_down")) {
                leftServoPos -= 0.01;
            } else if (gamepad1.dpad_right  && risingEdge.checkButton(currentGamepad1, "dpad_right")) {
                rightServoPos += 0.01;
            } else if (gamepad1.dpad_left && risingEdge.checkButton(currentGamepad1, "dpad_left")) {
                rightServoPos -= 0.01;
            }

            if (gamepad1.square || gamepad1.cross) {
                subsystem.leftServo.setPosition(leftServoPos);
            } else if (gamepad1.triangle || gamepad1.circle) {
                subsystem.rightServo.setPosition(rightServoPos);
            }

            leftServoPos = Math.max(Math.min(leftServoPos, 1), 0);

            rightServoPos = Math.max(Math.min(rightServoPos, 1), 0);

            currentGamepad1.copy(gamepad1);

            telemetry.addData("leftServo getPosition", new BigDecimal((String.valueOf(subsystem.leftServo.getPosition()))).setScale(2, RoundingMode.HALF_UP).doubleValue());
            telemetry.addData("rightServo getPosition", new BigDecimal((String.valueOf(subsystem.rightServo.getPosition()))).setScale(2, RoundingMode.HALF_UP).doubleValue());
            telemetry.addData("leftServoPos", new BigDecimal((String.valueOf(leftServoPos))).setScale(2, RoundingMode.HALF_UP).doubleValue());
            telemetry.addData("rightServoPos", new BigDecimal((String.valueOf(rightServoPos))).setScale(2, RoundingMode.HALF_UP).doubleValue());

            telemetry.update();
        }
    }
}