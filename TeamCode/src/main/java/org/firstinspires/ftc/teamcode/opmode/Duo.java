package org.firstinspires.ftc.teamcode.opmode;

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
public class Duo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        deposit.leftWrist.setPosition(0.5);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            deposit.leftWrist.setPosition(1);
        }
    }
}