package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@Config
@TeleOp(name = "Shooter And Spindexer TEst", group = "Tuning")
public class ShooterAndSpindexerTuningOp extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    private SpindexerSubsystem spindexerSubsystem;

    // --- DASHBOARD VARIABLES ---
    public static double p = -0.008;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = -0.00052;

    public enum Mode {
        RAW_TICKS,    // Tune pure motor physics
        LINEAR_SPEED  // Tune game logic (inches/sec)
    }
    public static Mode tuningMode = Mode.RAW_TICKS;

    // Set this if using RAW_TICKS
    public static double targetTicks = 0;

    // Set this if using LINEAR_SPEED
    public static double targetSpeedInches = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap);
        spindexerSubsystem.set(75);

        telemetry.addLine("Initialized.");
        telemetry.addLine("1. Open Dashboard.");
        telemetry.addLine("2. Set 'f' (Feedforward) first.");
        telemetry.addLine("3. Increase Target.");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            spindexerSubsystem.moveSpindexerBy(120);
        }
        // 1. Update PIDF from Dashboard
        shooterSubsystem.setPIDF(p, i, d, f);

        // 2. Set Target based on Mode
        switch (tuningMode) {
            case RAW_TICKS:
                shooterSubsystem.setTargetTicks(targetTicks);
                break;
            case LINEAR_SPEED:
                shooterSubsystem.setTargetLinearSpeed(targetSpeedInches);
                break;
        }

        // 3. Run Subsystem Loop (Calculates PID)
        shooterSubsystem.periodic();
        spindexerSubsystem.periodic();

        // 4. Telemetry for Graphing
        double target = shooterSubsystem.getTargetTicks();
        double actual = shooterSubsystem.getActualVelocity();

        telemetry.addData("Target (Ticks/Sec)", target);
        telemetry.addData("Actual (Ticks/Sec)", actual);
        telemetry.addData("Error", target - actual);

        // Helpful helper to see if math works
        if(tuningMode == Mode.LINEAR_SPEED) {
            telemetry.addData("Target Linear (in/s)", targetSpeedInches);
        }

        telemetry.update();
    }
}