package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter kV Finder")
public class ShooterKVFinder extends OpMode {

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private double maxVelocity = 0;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotor.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            shooter1.setPower(1.0);
            shooter2.setPower(1.0);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            maxVelocity = 0;
        }

        // Usa el mismo motor que usarás para el PID
        double velocity = shooter2.getVelocity();

        if (velocity > maxVelocity) {
            maxVelocity = velocity;
        }

        telemetry.addLine("Mantén presionado A");
        telemetry.addData("Velocidad Actual", velocity);
        telemetry.addData("Velocidad Máxima", maxVelocity);
        telemetry.addData("kV Aproximado", 0.95 / Math.max(maxVelocity, 1));
        telemetry.update();
    }
}