package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
<<<<<<< Updated upstream
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class armTester extends OpMode {
=======
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class armTester extends LinearOpMode {
>>>>>>> Stashed changes
    public static double leftArmPos = 0.5;
    public static double rightArmPos = 0.5;
    public static boolean USE_DASHBOARD = false;

<<<<<<< Updated upstream
    Deposit deposit = new Deposit(hardwareMap);
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit.leftArm.setPosition(leftArmPos);
        deposit.rightArm.setPosition(rightArmPos);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            deposit.leftArm.setPosition(leftArmPos);
            deposit.rightArm.setPosition(rightArmPos);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            leftArmPos += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            leftArmPos -= 0.01;
        } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
            rightArmPos += 0.01;
        } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
            rightArmPos -= 0.01;
        }

        if (gamepad1.square || gamepad1.cross) {
            deposit.leftArm.setPosition(leftArmPos);
        } else if (gamepad1.triangle || gamepad1.circle) {
            deposit.rightArm.setPosition(rightArmPos);
        }

        leftArmPos = Math.max(Math.min(leftArmPos, 1), 0);

        rightArmPos = Math.max(Math.min(rightArmPos, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("leftArm getPosition", round(deposit.leftArm.getPosition(), 2));
        telemetry.addData("rightArm getPosition", round(deposit.rightArm.getPosition(), 2));
        telemetry.addData("leftArmPos", round(leftArmPos, 2));
        telemetry.addData("rightArmPos", round(rightArmPos, 2));
        telemetry.update();
=======
    @Override
    public void runOpMode() throws InterruptedException {
        Servo leftArm = hardwareMap.get(Servo.class, "leftArm");
        Servo rightArm = hardwareMap.get(Servo.class, "rightArm");

        Gamepad currentGamepad1 = new Gamepad();

        leftArm.setPosition(leftArmPos);
        rightArm.setPosition(rightArmPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (USE_DASHBOARD){
                leftArm.setPosition(leftArmPos);
                rightArm.setPosition(rightArmPos);
            } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
                leftArmPos += 0.01;
            } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
                leftArmPos -= 0.01;
            } else if (gamepad1.dpad_right  && checkButton(currentGamepad1, "dpad_right")) {
                rightArmPos += 0.01;
            } else if (gamepad1.dpad_left && checkButton(currentGamepad1, "dpad_left")) {
                rightArmPos -= 0.01;
            }

            if (gamepad1.square || gamepad1.cross) {
                leftArm.setPosition(leftArmPos);
            } else if (gamepad1.triangle || gamepad1.circle) {
                rightArm.setPosition(rightArmPos);
            }

            leftArmPos = Math.max(Math.min(leftArmPos, 1), 0);

            rightArmPos = Math.max(Math.min(rightArmPos, 1), 0);

            currentGamepad1.copy(gamepad1);

            telemetry.addData("leftArm getPosition", round(leftArm.getPosition(), 2));
            telemetry.addData("rightArm getPosition", round(rightArm.getPosition(), 2));
            telemetry.addData("leftArmPos", round(leftArmPos, 2));
            telemetry.addData("rightArmPos", round(rightArmPos, 2));
            telemetry.update();
        }
>>>>>>> Stashed changes
    }
}