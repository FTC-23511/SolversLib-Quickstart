package org.firstinspires.ftc.teamcode.tuning.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@TeleOp
public class armTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        double leftWristPos = 0.5;
        double rightWristPos = 0.5;


        deposit.leftWrist.setPosition(leftWristPos);
        deposit.rightWrist.setPosition(rightWristPos);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                leftWristPos += 0.01;
            }

            if (gamepad1.dpad_down) {
                leftWristPos -= 0.01;
            }

            if (gamepad1.dpad_left) {
                rightWristPos += 0.01;
            }

            if (gamepad1.dpad_right) {
                rightWristPos -= 0.01;
            }

            if (gamepad1.square) {
                deposit.leftWrist.setPosition(leftWristPos);
            }

            if (gamepad1.circle) {
                deposit.rightWrist.setPosition(rightWristPos);
            }

            telemetry.addData("leftWrist getPosition", deposit.leftWrist.getPosition());
            telemetry.addData("rightWrist getPosition", deposit.rightWrist.getPosition());
            telemetry.addData("leftWristPos", leftWristPos);
            telemetry.addData("rightWristPos", rightWristPos);
            telemetry.update();
        }
    }
}