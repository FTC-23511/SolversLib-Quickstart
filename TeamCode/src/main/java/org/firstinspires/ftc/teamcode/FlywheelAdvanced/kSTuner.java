package org.firstinspires.ftc.teamcode.FlywheelAdvanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class kSTuner extends OpMode {

    Flywheel flywheel = new Flywheel();

    public static double kS = 0.45;

    @Override
    public void init() {
        flywheel.init(hardwareMap);
    }

    @Override
    public void loop() {
        flywheel.setMotorPower(kS);

        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("Ticks per sec", flywheel.getTicksPerSec());
        telemetry.update(); // Asegúrate de actualizar la telemetría
    }
}