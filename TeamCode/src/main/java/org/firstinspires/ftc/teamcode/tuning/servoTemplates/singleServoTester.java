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
public class singleServoTester extends OpMode {
    public static double servoPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    Subsystem subsystem = new Subsystem(hardwareMap);
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        subsystem.centerServo.setPosition(servoPos);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            subsystem.centerServo.setPosition(servoPos);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            servoPos += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            servoPos -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            subsystem.centerServo.setPosition(servoPos);
        }

        servoPos = Math.max(Math.min(servoPos, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerServo getPosition", round(subsystem.centerServo.getPosition(), 2));
        telemetry.addData("centerServoPos",round(servoPos, 2));
        telemetry.update();
    }
}