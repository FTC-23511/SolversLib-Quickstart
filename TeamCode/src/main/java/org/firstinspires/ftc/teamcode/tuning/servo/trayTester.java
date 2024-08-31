package org.firstinspires.ftc.teamcode.tuning.servo.sample;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.System.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Config
@TeleOp
public class trayTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final Robot robot = Robot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = Globals.OpModeType.TELEOP;
        driveMode = Globals.DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.tray.setPosition(CENTER_SERVO_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD) {
            robot.tray.setPosition(CENTER_SERVO_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_SERVO_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            CENTER_SERVO_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            CENTER_SERVO_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross || gamepad1.triangle || gamepad1.circle) {
            robot.tray.setPosition(CENTER_SERVO_POS);
        }

        CENTER_SERVO_POS = Math.max(Math.min(CENTER_SERVO_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("tray getPosition", robot.tray.getPosition());
        telemetry.addData("trayPos", round(CENTER_SERVO_POS, 2));
        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}