package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp
public class PID_test extends OpMode {
    DcMotorEx pivot;

    public static double p = 0.0, i = 0, d = 0.000;
    public static double f = 0.00;
    public static int target = 0;

    Gamepad lastGamepad1 = new Gamepad();
    PIDFController controller;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(p, i, d, f);

        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
    }

    @Override
    public void loop() {
        int armPos = pivot.getCurrentPosition();

        if (gamepad1.a && !lastGamepad1.a) {
            target = 500;
        } else if (gamepad1.b && !lastGamepad1.b) {
            target = 700;
        } else if (gamepad1.x && !lastGamepad1.x) {
            target = 1500;
        }

        pivot.setPower(controller.calculate(armPos, target));

        telemetry.addData("position", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
