package org.firstinspires.ftc.teamcode.tuning.servoTemplates;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.servoTemplates.Subsystem;

//@Config
//@TeleOp
public class doubleServoTester extends OpMode {
    public static double leftServoPos = 0.5;
    public static double rightServoPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    Subsystem subsystem = new Subsystem(hardwareMap);
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystem.leftServo.setPosition(leftServoPos);
        subsystem.rightServo.setPosition(rightServoPos);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            subsystem.leftServo.setPosition(leftServoPos);
            subsystem.rightServo.setPosition(rightServoPos);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            leftServoPos += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            leftServoPos -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            rightServoPos += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
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

        telemetry.addData("leftServo getPosition", round(subsystem.leftServo.getPosition(), 2));
        telemetry.addData("rightServo getPosition", round(subsystem.rightServo.getPosition(), 2));
        telemetry.addData("leftServoPos", round(leftServoPos, 2));
        telemetry.addData("rightServoPos", round(rightServoPos, 2));
        telemetry.update();
    }
}