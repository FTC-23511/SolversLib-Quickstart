package org.firstinspires.ftc.teamcode.tuning.PIDF;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp(name = "LaunchMotorTuner")
public class LaunchMotorTuner extends CommandOpMode {
    // D, 0.000_
    // F, 0.000_
    // I, 0
    // maxPowerConstant, 1.0
    // P, 0.00_

    public static double P = 0.00;
    public static double I = 0;
    public static double D = 0.000;
    public static double F = 0.000;
    public static double targetVel = 0.0;
    public static double posTolerance = 0;

    private static final PIDFController launcherPIDF = new PIDFController(P, I, D, F);

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;
        launcherPIDF.setTolerance(posTolerance, 0);

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            timer = new ElapsedTime();
        }

        double motorVel = robot.launchMotors.getVelocity();

        launcherPIDF.setP(P);
        launcherPIDF.setI(I);
        launcherPIDF.setD(D);
        launcherPIDF.setF(F);

        launcherPIDF.setTolerance(posTolerance, 0);

        launcherPIDF.setSetPoint(targetVel);

        double maxPower = (F * motorVel);
        double power = Range.clip(launcherPIDF.calculate(targetVel, motorVel), -maxPower, maxPower);

        robot.launchMotors.set(power);

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("power", power);
        telemetryData.addData("target velocity", targetVel);
        telemetryData.addData("target velocity", motorVel);
        telemetryData.addData("encoder position", robot.launchMotors.getPositions());

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        robot.pinpoint.update();
        telemetryData.update();
    }
    
    @Override
    public void end() {
        Log.v("P", String.valueOf(P));
        Log.v("I", String.valueOf(I));
        Log.v("D", String.valueOf(D));
        Log.v("F", String.valueOf(F));
        Log.v("posTolerance", String.valueOf(posTolerance));
    }
}