package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
@TeleOp(name = "Shooter pid tuning", group = " Tuning ")
public class ShooterPIDTuning extends OpMode {

    public static double p = 0.0, i = 0.0, d = 0.0;
    public static double s = 0.0, v = 0.0, a = 0.0;
    public static double targetVelocity = 100; // encoder counts per second

    Motor shooter;
    private ElapsedTime deltaTime = new ElapsedTime();

    @Override
    public void init() {
        shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_312);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter.setRunMode(Motor.RunMode.VelocityControl);
        shooter.setVeloCoefficients(p, i, d);
        shooter.setFeedforwardCoefficients(s, v, a);
        shooter.set(0.0);

        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        deltaTime.reset();
    }

    @Override
    public void loop() {
        shooter.setVeloCoefficients(p, i, d);
        shooter.setFeedforwardCoefficients(s, v, a);

        shooter.set(targetVelocity);

        // Telemetry
        telemetry.addData("currentpos", shooter.getCurrentPosition());
        telemetry.addData("Velocity", shooter.getCorrectedVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.update();

        // Prepare for next loop
        deltaTime.reset();
    }
}
