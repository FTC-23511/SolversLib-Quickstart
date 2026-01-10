package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.seattlesolvers.solverslib.hardware.ServoEx;

@Config
@TeleOp(name = "Gate Tuning", group = "Tuning")
public class GatePositionTuningOp extends OpMode {

    // Edit this in Dashboard to move the servo
    public static double targetPos = 0.75;

    private ServoEx gate;
    private AnalogInput gateEncoder;

    @Override
    public void init() {
        // Initialize Telemetry for Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware using the same names as your Subsystem
        gate = new ServoEx(hardwareMap, "gate");
        gateEncoder = hardwareMap.get(AnalogInput.class, "gateEncoder");

        // Match the inversion setting from your Subsystem
        gate.setInverted(true);

        telemetry.addLine("Initialized. Use Dashboard to tune 'targetPos'.");
    }

    @Override
    public void loop() {
        // Set the servo to the dashboard variable
        gate.set(targetPos);

        double currentVoltage = gateEncoder.getVoltage();

        // Telemetry
        telemetry.addData("Target Servo Pos", targetPos);
        telemetry.addData("Current Voltage", currentVoltage);

        telemetry.addLine("--- Tuning Guide ---");
        telemetry.addData("If Gate is UP, set UP_VOLTAGE to", currentVoltage);
        telemetry.addData("If Gate is DOWN, set DOWN_VOLTAGE to", currentVoltage);

        telemetry.update();
    }
}