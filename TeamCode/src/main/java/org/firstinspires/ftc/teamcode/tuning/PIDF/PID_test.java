package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class PID_test extends OpMode {
    public static int setPoint = 0;

    // D, 0.0004
    // F, 0.0006
    // I, 0
    // maxPowerConstant, 0.5
    // P, 0.011
    /*

    1. Make sure all values are 0!
    2. Mov
     */

    public static double p = 0.00;
    public static double i = 0;
    public static double d = 0.000;
    public static double f = 0.000;
    public static double maxPowerConstant = 1.0;

    private static final PIDFController slidePIDF = new PIDFController(p,i,d, f);
    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    int liftPos = robot.liftEncoder.getPosition();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        slidePIDF.setTolerance(10, 10);

        telemetry.addData("encoder position", liftPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("max power", (f * liftPos) + maxPowerConstant);

        robot.liftEncoder.reset();
    }

    @Override
    public void loop() {
        timer.reset();

        liftPos = robot.liftEncoder.getPosition();

        slidePIDF.setP(p);
        slidePIDF.setI(i);
        slidePIDF.setD(d);
        slidePIDF.setF(f);

        slidePIDF.setSetPoint(setPoint);
        double maxPower = (f * liftPos) + maxPowerConstant;

        double power = Range.clip(slidePIDF.calculate(liftPos, setPoint), -maxPower, maxPower);
        robot.liftRight.setPower(power);
        robot.liftLeft.setPower(power);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", liftPos);
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("motorPower", power);
        telemetry.addData("max power", maxPower);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}