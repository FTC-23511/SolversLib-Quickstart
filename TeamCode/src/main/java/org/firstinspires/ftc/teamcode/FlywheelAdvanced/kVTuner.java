package org.firstinspires.ftc.teamcode.FlywheelAdvanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class kVTuner extends OpMode {
    Flywheel flywheel = new Flywheel();

    public static double kV = 0.00024;
    public static double Velocity = 400;
    public static double kP = 0.001;
    public static double kS = 0.3;
    public  static double goalRPM = 1500;

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()){
            flywheel.Transfer(Velocity);
        } else if (gamepad1.bWasPressed()) {
            flywheel.Transfer(0);
        }
        double feedFordward = (kV * goalRPM) + kS;
        double error = goalRPM - flywheel.getRPM();
        double feedback = error * kP;
        flywheel.setMotorPower(feedFordward + feedback);

        telemetry.addData("kp", "%.6f", kP);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("error", error);
        telemetry.addData("Transfer", flywheel.VelocityTranfer());
        telemetry.addData("Ticks per sec", flywheel.getTicksPerSec());
        telemetry.update(); // Asegúrate de actualizar la telemetría
    }
}
