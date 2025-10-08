package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Config
@TeleOp (name = "Shooter pid tuning", group = " Tuning ")
public class ShooterPIDTuning extends OpMode {
    private PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0;

    public static int target=0;

    private DcMotor shooter;

    ElapsedTime deltaTime = new ElapsedTime();
    int lastPos = 0;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotor.class, "shooter");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int shooterPos = shooter.getCurrentPosition();
        double velocity = deltaTime.time() == 0 ? 0 : (lastPos - shooterPos) / deltaTime.time();
        double pid = controller.calculate(shooterPos, target);
        double ff = 0.0;

        double power = pid + ff;

        shooter.setPower(power);
        telemetry.addData("velocity ", velocity);
        telemetry.addData("target ", target);
        telemetry.update();
        deltaTime.reset();
        lastPos = shooterPos;
    }
}
