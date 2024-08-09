package org.firstinspires.ftc.teamcode.tuning.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

@Config
@TeleOp
public class dashWristTester extends LinearOpMode {

    public static double wristPos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        Deposit deposit = new Deposit(hardwareMap);

        deposit.wrist.setPosition(wristPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            deposit.wrist.setPosition(wristPos);

            telemetry.addData("wrist getPosition", deposit.wrist.getPosition());
            telemetry.addData("wristPos", wristPos);

            telemetry.update();
        }
    }
}