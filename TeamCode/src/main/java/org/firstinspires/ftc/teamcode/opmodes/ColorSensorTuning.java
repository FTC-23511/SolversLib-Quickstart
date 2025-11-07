package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;

import java.util.Arrays;

@Config
@TeleOp(name = "Color Sensor Tuning ", group = "tuning")
public class ColorSensorTuning extends OpMode {
    private ColorSubsystem colorSensor;
    @Override
    public void init() {
        colorSensor = new ColorSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        float[] sensedColor = colorSensor.senseColor(1);
        boolean isGreen = colorSensor.checkIfGreen(1);
        boolean isPurple = colorSensor.checkIfPurple(1);

        telemetry.addData("sensed color hsv", Arrays.toString(sensedColor));
        telemetry.addData("detects green", isGreen);
        telemetry.addData("detects purple", isPurple);

    }
}
