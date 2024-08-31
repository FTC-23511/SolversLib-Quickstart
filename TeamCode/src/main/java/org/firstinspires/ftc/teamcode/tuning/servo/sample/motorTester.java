package org.firstinspires.ftc.teamcode.tuning.servo.sample;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@Photon
@Config
@TeleOp
public class motorTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    ExampleRobot exampleHardware = new ExampleRobot();
    Gamepad currentGamepad1 = new Gamepad();
    double CENTER_MOTOR_POWER = 0;
    @Override
    public void init() {
        exampleHardware.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        exampleHardware.motor.setPower(0);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            exampleHardware.motor.setPower(CENTER_MOTOR_POWER);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            CENTER_MOTOR_POWER += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            CENTER_MOTOR_POWER -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            exampleHardware.motor.setPower(CENTER_MOTOR_POWER);
        }
        else{
            exampleHardware.motor.setPower(0);
        }

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerMotor Power", exampleHardware.motor.getPower());
        telemetry.update();


    }
}