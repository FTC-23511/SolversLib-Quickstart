package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
@TeleOp(name = "Shooter pid tuning", group = " Tuning ")
public class ShooterPIDTuning extends OpMode {

    public static double p = 0.0015, i = 0.0, d = 0.0;
    public static double f = 0.00067;
    public static double targetVelocity = 2000; // ticks per second

    Motor shooter;
    private PIDFController flywheelController;
    private ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_312);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // Initialize custom PIDF controller (same as ShooterSubSystem)
        flywheelController = new PIDFController(p, i, d, f);
        flywheelController.setSetPoint(targetVelocity);

        deltaTime.reset();
    }

    @Override
    public void loop() {
        flywheelController.setPIDF(p, i, d, f);
        flywheelController.setSetPoint(targetVelocity);

        double currentVelocity = shooter.getCorrectedVelocity();
        double powerOutput = flywheelController.calculate(currentVelocity);

        shooter.set(powerOutput);

        // Telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Error", targetVelocity - currentVelocity);
        telemetry.addData("Output Power", powerOutput);
        telemetry.update();

        // Prepare for next loop
        deltaTime.reset();
    }
}
