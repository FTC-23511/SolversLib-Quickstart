package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class PID_test extends OpMode {
    public static int setPoint = 0;

    // 1. F, 0.0005
    // 2. P, 0.0065
    // 2. D, 0.0003
    /*
    1. Make sure all values are 0!
    2.





     */

    public static double p = 0.00, i = 0, d = 0.000;
    public static double f = 0.000;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);

    private final Robot robot = Robot.getInstance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(10, 10);

        telemetry.addData("encoder position", robot.liftEncoder.getPosition());
        telemetry.addData("setPoint", setPoint);

        robot.liftEncoder.reset();
    }

    @Override
    public void loop() {
        int liftPos = robot.liftEncoder.getPosition();

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);

        double power = slidePIDF.calculate(liftPos, setPoint);
        robot.liftRight.setPower(power);
        robot.liftLeft.setPower(power);

        telemetry.addData("encoder position", liftPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.update();

        robot.ControlHub.clearBulkCache();
    }
}
