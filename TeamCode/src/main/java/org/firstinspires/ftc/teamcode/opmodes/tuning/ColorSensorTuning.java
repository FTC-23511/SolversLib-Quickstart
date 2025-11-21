package org.firstinspires.ftc.teamcode.opmodes.tuning;

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
        float[] sensedColor1 = colorSensor.senseColorsHSV(1);
        boolean isGreen1 = colorSensor.checkIfGreen(1);
        boolean isPurple1 = colorSensor.checkIfGreen(1);

//        float[] sensedColor2 = colorSensor.senseColorsHSV(2);
//        boolean isGreen2 = ColorSensorsSubsystem.checkIfGreen(sensedColor2);
//        boolean isPurple2 = ColorSensorsSubsystem.checkIfPurple(sensedColor2);

        telemetry.addData("#1 sensed color hsv ", Arrays.toString(sensedColor1));
        telemetry.addData("#1 detects green ", isGreen1);
        telemetry.addData("#1 detects purple ", isPurple1);

//        telemetry.addData("#2 sensed color hsv ", Arrays.toString(sensedColor2));
//        telemetry.addData("#2 detects green ", isGreen2);
//        telemetry.addData("#2 detects purple ", isPurple2);

    }
}
