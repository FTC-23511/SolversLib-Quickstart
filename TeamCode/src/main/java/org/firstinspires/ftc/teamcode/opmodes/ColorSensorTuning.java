package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubSystem;

public class ColorSensorTuning extends OpMode {
    private ColorSubSystem colorSensor;
    @Override
    public void init() {
        colorSensor = new ColorSubSystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        float[] sensedColor = colorSensor.rgbToHsv(colorSensor.senseColor());
        boolean isGreen = colorSensor.checkIfGreen();
        boolean isPurple = colorSensor.checkIfPurple();

        telemetry.addData("sensed color hsv", sensedColor);
        telemetry.addData("detects green", isGreen);
        telemetry.addData("detects purple", isPurple);

    }
}
