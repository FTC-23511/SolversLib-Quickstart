package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;

import java.util.Arrays;

@Config
@TeleOp(name = "Color Sensor Tuning ", group = "tuning")
public class ColorSensorTuning extends OpMode {
    private ColorSensorsSubsystem colorSensor;
    @Override
    public void init() {
        colorSensor = new ColorSensorsSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        //TODO: Make this work for both color sensors
        float[] sensedColor = colorSensor.senseColorsHSV();
        boolean isGreen = colorSensor.checkIfGreen();
        boolean isPurple = colorSensor.checkIfPurple();

        telemetry.addData("sensed color hsv", Arrays.toString(sensedColor));
        telemetry.addData("detects green", isGreen);
        telemetry.addData("detects purple", isPurple);

    }
}
