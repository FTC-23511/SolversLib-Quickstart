package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class wristTester extends LinearOpMode {
    public static double wristPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();

        deposit.wrist.setPosition(wristPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (USE_DASHBOARD){
                deposit.wrist.setPosition(wristPos);
            } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
                wristPos += 0.01;
            } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
                wristPos -= 0.01;
            }

            if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
                deposit.wrist.setPosition(wristPos);
            }

            wristPos = Math.max(Math.min(wristPos, 1), 0);

            currentGamepad1.copy(gamepad1);

            telemetry.addData("wrist getPosition", round(deposit.wrist.getPosition(), 2));
            telemetry.addData("wristPos", round(wristPos, 2));
            telemetry.update();
        }
    }
}