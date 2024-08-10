package org.firstinspires.ftc.teamcode.tuning.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class dashArmTester extends LinearOpMode {

    public static double leftWristPos = 0.5;
    public static double rightWristPos = 0.5;

    public static boolean moveLeft = false;
    public static boolean moveRight = false;
    public static boolean moveBoth = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        deposit.leftArm.setPosition(leftWristPos);
        deposit.rightArm.setPosition(rightWristPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (moveLeft) {
                deposit.leftArm.setPosition(leftWristPos);

                moveLeft = false;
            }

            if (moveRight) {
                deposit.rightArm.setPosition(rightWristPos);

                moveRight = false;
            }

            if (moveBoth) {
                deposit.leftArm.setPosition(leftWristPos);
                deposit.rightArm.setPosition(rightWristPos);

                moveBoth = false;
            }

            telemetry.addData("leftWrist getPosition", deposit.leftArm.getPosition());
            telemetry.addData("rightWrist getPosition", deposit.rightArm.getPosition());

            telemetry.addData("leftWristPos", leftWristPos);
            telemetry.addData("rightWristPos", rightWristPos);

            telemetry.addData("moveLeft", moveLeft);
            telemetry.addData("moveRight", moveRight);
            telemetry.addData("moveBoth", moveBoth);
            telemetry.update();
        }
    }
}