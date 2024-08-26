package org.firstinspires.ftc.teamcode.tuning.servo.sample;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CENTER_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleHardware;

//@Config
//@TeleOp
public class singleServoTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    ExampleHardware exampleHardware = new ExampleHardware();
    Gamepad currentGamepad1 = new Gamepad();
    
    @Override
    public void init() {
        exampleHardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        exampleHardware.centerServo.setPosition(CENTER_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            exampleHardware.centerServo.setPosition(CENTER_SERVO_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_SERVO_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            exampleHardware.centerServo.setPosition(CENTER_SERVO_POS);
        }

        CENTER_SERVO_POS = Math.max(Math.min(CENTER_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerServo getPosition", round(exampleHardware.centerServo.getPosition(), 2));
        telemetry.addData("centerServoPos",round(CENTER_SERVO_POS, 2));
        telemetry.update();
    }
}