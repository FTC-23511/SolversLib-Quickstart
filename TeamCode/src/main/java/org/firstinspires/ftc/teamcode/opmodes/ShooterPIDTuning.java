package org.firstinspires.ftc.teamcode.opmodes;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;

@Config
@TeleOp(name = "Shooter pid tuning", group = " Tuning ")
public class ShooterPIDTuning extends OpMode {

    private PIDController controller;

    public static double p = 0.0000, i = 0.000000, d = 0.000000;
    public static double ff = 0.0;
    public static double targetVelocity = 100; // encoder counts per second

    private DcMotor shooter;
    private ElapsedTime deltaTime = new ElapsedTime();
    private int lastPos = 0;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        lastPos = shooter.getCurrentPosition();
        deltaTime.reset();
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        // Calculate current velocity
        int currentPos = shooter.getCurrentPosition();
        double dt = deltaTime.seconds();
        double velocity = 0;
        if (dt > 0.0001) { // avoid divide by zero
            velocity = (double)(currentPos - lastPos) / dt;
        }

        // PID calculation
        double pid = controller.calculate(velocity, targetVelocity);

        // Feedforward + PID
        double power = pid + ff;

        shooter.setPower(power);

        // Telemetry
        telemetry.addData("currentpos", currentPos);
        telemetry.addData("lastpos", lastPos);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("dt", dt);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Power", power);
        telemetry.update();

        // Prepare for next loop
        lastPos = currentPos;
        deltaTime.reset();
    }
}
