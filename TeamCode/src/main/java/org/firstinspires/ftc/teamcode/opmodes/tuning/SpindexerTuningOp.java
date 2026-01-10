package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@Config
@TeleOp(name = "Spindexer Tuner", group = "Tuning")
public class SpindexerTuningOp extends LinearOpMode {

    private SpindexerSubsystem spindexer;
    private AnalogInput absoluteEncoder;


    // DASHBOARD VARIABLES (Edit these in the browser)
    // ------------------------------------------------
    public static double TARGET_POS = 0; // Target in degrees

    // Default PID (Matches your subsystem defaults)
    public static double p = 0.0159;
    public static double i = 0;
    public static double d = 0.0000114;
    public static double f = 0;

    // Use this to check your voltage scaling
    // If you spin 360 and this doesn't match the subsystem's math, adjust your code.
    public static double VOLTAGE_SCALAR = 3.2;
    // ------------------------------------------------

    enum Mode {
        CALIBRATE, // Motor is float, read voltages manually
        RUN        // PID is active
    }

    Mode currentMode = Mode.CALIBRATE;
    boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Telemetry for Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spindexer = new SpindexerSubsystem(hardwareMap);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "spindexerAnalog");


        // Access the raw motor to float it during calibration
        // Note: We need to cast or access the motor from hardwareMap if it's private in subsystem,
        // but ideally your subsystem handles 'stop' or 'float'.
        // For this example, we assume we can set power to 0 in the loop.

        telemetry.addLine("Ready to tune.");
        telemetry.addLine("Press A to toggle between CALIBRATE (Manual spin) and RUN (PID).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Toggle Mode Logic
            boolean currentA = gamepad1.a;
            if (currentA && !lastA) {
                if (currentMode == Mode.CALIBRATE) {
                    currentMode = Mode.RUN;
                } else {
                    currentMode = Mode.CALIBRATE;
                    // Reset target to current position so it doesn't snap wildly when re-enabling
                    TARGET_POS = spindexer.getCurrentPosition();
                }
            }
            lastA = currentA;

            // 2. Handle Modes
            if (currentMode == Mode.CALIBRATE) {
                // Stop motor / allow free spin
                // (You might need to add a setPower(0) method to your subsystem if set() runs PID)
                // Assuming spindexer.set(getCurrent) or similar keeps it still,
                // but for calibration we want 0 power.
                // NOTE: If your subsystem periodic() always writes power, you might fight it.
                // Ideally, add a method spindexer.stop() that sets power 0.

                // For now, we will use the trick of setting target to current,
                // but to truly float you need to modify subsystem or access motor directly.
                // Let's assume you add spindexer.setMotorPower(0) or similar.

                telemetry.addData("Mode", "CALIBRATE (Spin manually)");
                telemetry.addData("Instructions", "Rotate spindexer to 0, 90, 180, 270, 360");
                telemetry.addData("Raw Voltage", absoluteEncoder.getVoltage());

                // Show what the code THINKs the angle is vs Raw Voltage
                double calculatedAngle = (absoluteEncoder.getVoltage() / VOLTAGE_SCALAR) * 360;
                telemetry.addData("Calc Angle (Linear)", calculatedAngle);

            } else {
                // RUN MODE
                telemetry.addData("Mode", "RUN (PID Active)");

                // Update PID from Dashboard
                spindexer.setPIDCoefficients(p, i, d, f);

                // Update Target from Dashboard
                spindexer.set(TARGET_POS);

                // Run the periodic loop (calculates PID and sets power)
                spindexer.periodic();

                // Dashboard Graphing Data
                telemetry.addData("Target", TARGET_POS);
                telemetry.addData("Actual", spindexer.getCurrentPosition());
                telemetry.addData("Error", TARGET_POS - spindexer.getCurrentPosition());
                telemetry.addData("Motor Output", spindexer.getOutput());
            }

            telemetry.update();
        }
    }
}