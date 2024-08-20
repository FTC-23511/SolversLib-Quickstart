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
public class armTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    RobotHardware robotHardware = new RobotHardware();
    Gamepad currentGamepad1 = new Gamepad();
    @Override

    public void init() {
        robotHardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware.leftArm.setPosition(ARM_BACKDROP_POS);
        robotHardware.rightArm.setPosition(ARM_BACKDROP_POS);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            robotHardware.leftArm.setPosition(ARM_BACKDROP_POS);
            robotHardware.rightArm.setPosition(ARM_BACKDROP_POS);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            ARM_BACKDROP_POS += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            ARM_BACKDROP_POS -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            ARM_BACKDROP_POS += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            ARM_BACKDROP_POS -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross) {
            robotHardware.leftArm.setPosition(ARM_BACKDROP_POS);
        } else if (gamepad1.triangle || gamepad1.circle) {
            robotHardware.rightArm.setPosition(ARM_BACKDROP_POS);
        }

        ARM_BACKDROP_POS = Math.max(Math.min(ARM_BACKDROP_POS, 1), 0);
        ARM_BACKDROP_POS = Math.max(Math.min(ARM_BACKDROP_POS, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("leftArm getPosition", round(robotHardware.leftArm.getPosition(), 2));
        telemetry.addData("rightArm getPosition", round(robotHardware.rightArm.getPosition(), 2));
        telemetry.addData("leftArm", round(ARM_BACKDROP_POS, 2));
        telemetry.addData("rightArm", round(ARM_BACKDROP_POS, 2));
        telemetry.update();
    }
}