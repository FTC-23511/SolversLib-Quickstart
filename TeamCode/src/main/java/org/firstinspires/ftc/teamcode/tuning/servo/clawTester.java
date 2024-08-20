package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;
import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
@TeleOp
public class clawTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    RobotHardware robotHardware = new RobotHardware();
    Gamepad currentGamepad1 = new Gamepad();
    @Override

    public void init() {
        robotHardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware.leftClaw.setPosition(LEFT_CLAW_OPEN_POS);
        robotHardware.rightClaw.setPosition(RIGHT_CLAW_OPEN_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            robotHardware.leftClaw.setPosition(LEFT_CLAW_OPEN_POS);
            robotHardware.rightClaw.setPosition(RIGHT_CLAW_OPEN_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            LEFT_CLAW_OPEN_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            LEFT_CLAW_OPEN_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            RIGHT_CLAW_OPEN_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            RIGHT_CLAW_OPEN_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross) {
            robotHardware.leftClaw.setPosition(LEFT_CLAW_OPEN_POS);
        } else if (gamepad1.triangle || gamepad1.circle) {
            robotHardware.rightClaw.setPosition(RIGHT_CLAW_OPEN_POS);
        }

        LEFT_CLAW_OPEN_POS = Math.max(Math.min(LEFT_CLAW_OPEN_POS, 1), 0);
        RIGHT_CLAW_OPEN_POS = Math.max(Math.min(RIGHT_CLAW_OPEN_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("leftClaw getPosition", round(robotHardware.leftClaw.getPosition(), 2));
        telemetry.addData("rightClaw getPosition", round(robotHardware.rightClaw.getPosition(), 2));
        telemetry.addData("leftClawPos", round(LEFT_CLAW_OPEN_POS, 2));
        telemetry.addData("rightClawPos", round(RIGHT_CLAW_OPEN_POS, 2));
        telemetry.update();
    }
}