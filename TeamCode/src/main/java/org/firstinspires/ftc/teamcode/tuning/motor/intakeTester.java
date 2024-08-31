package org.firstinspires.ftc.teamcode.tuning.motor;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.System.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Photon
@Config
@TeleOp
public class intakeTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    private final Robot robot = Robot.getInstance();

    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intakeMotor.setPower(CENTER_MOTOR_POWER);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            robot.intakeMotor.setPower(CENTER_MOTOR_POWER);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_MOTOR_POWER += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_MOTOR_POWER -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            robot.intakeMotor.setPower(CENTER_MOTOR_POWER);
        }
        else{
            robot.intakeMotor.setPower(0);
        }

        currentGamepad1.copy(gamepad1);

        telemetry.addData("intakeMotor Power", robot.intakeMotor.getPower());
        telemetry.update();
    }
}