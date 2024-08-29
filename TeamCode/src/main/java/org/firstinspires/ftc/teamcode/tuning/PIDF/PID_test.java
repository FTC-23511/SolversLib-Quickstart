package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class PID_test extends OpMode {

    public static int target = 0;

    public static double p = 0.0, i = 0, d = 0.000;
    public static double f = 0.00;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);

    private final Robot robot = Robot.getInstance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(10, 10);
    }

    @Override
    public void loop() {
        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setD(f);

        slidePIDF.setSetPoint(target);

        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        robot.liftRight.setPower(power);
        robot.liftLeft.setPower(power);

        telemetry.addData("position", robot.liftEncoder.getPosition());
        telemetry.addData("target", target);
        telemetry.update();

        robot.ControlHub.clearBulkCache();
    }
}
