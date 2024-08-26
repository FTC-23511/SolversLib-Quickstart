package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.LEFT_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.RIGHT_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleHardware;

@Config
@TeleOp
public class wristTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    ExampleHardware exampleHardware = new ExampleHardware();
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        exampleHardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        exampleHardware.leftServo.setPosition(LEFT_SERVO_POS);
        exampleHardware.rightServo.setPosition(RIGHT_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            exampleHardware.leftServo.setPosition(LEFT_SERVO_POS);
            exampleHardware.rightServo.setPosition(RIGHT_SERVO_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            LEFT_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            LEFT_SERVO_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            RIGHT_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            RIGHT_SERVO_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross) {
            exampleHardware.leftServo.setPosition(LEFT_SERVO_POS);
        } else if (gamepad1.triangle || gamepad1.circle) {
            exampleHardware.rightServo.setPosition(RIGHT_SERVO_POS);
        }

        LEFT_SERVO_POS = Math.max(Math.min(LEFT_SERVO_POS, 1), 0);
        RIGHT_SERVO_POS = Math.max(Math.min(RIGHT_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("leftServo getPosition", round(exampleHardware.leftServo.getPosition(), 2));
        telemetry.addData("rightServo getPosition", round(exampleHardware.rightServo.getPosition(), 2));
        telemetry.addData("leftServoPos", round(LEFT_SERVO_POS, 2));
        telemetry.addData("rightServoPos", round(RIGHT_SERVO_POS, 2));
        telemetry.update();
    }
}