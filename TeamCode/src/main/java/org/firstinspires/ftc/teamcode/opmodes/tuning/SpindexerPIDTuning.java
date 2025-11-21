package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDController;

@Config
@TeleOp(name = "Spindexer PID Tuning Absolute", group = "tuning")
public class SpindexerPIDTuning extends OpMode {
    private PIDController controller;

    // PID constants to tune
    public static double p = 0.0159, i = 0.0, d = 0.0000114;
    public static double clampPower = 0.4;
    public static double targetDeg = 0; // target in degrees
    public static double offset = 0;    // analog offset

    private DcMotor spindexer;
    private AnalogInput analogInput;
    private double currentDeg = 0;
    private double output = 0;
    private double lastError = 0;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        analogInput = hardwareMap.get(AnalogInput.class, "spindexerAnalog");

        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        currentDeg = getAbsolutePosition();
        targetDeg = currentDeg; // start at current position
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        currentDeg = getAbsolutePosition();

        // Compute shortest-path error [-180, 180]
        double error = targetDeg - currentDeg;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        // Optional derivative term manually (if PIDController doesn't do it)
        double derivative = error - lastError;
        lastError = error;

        output = controller.calculate(0, error); // simple P controller
        output = clamp(output, -clampPower, clampPower);

        // Deadband to reduce jitter
        if (Math.abs(output) < 0.02) output = 0;

        spindexer.setPower(output);

        // Telemetry
        telemetry.addData("Analog Deg", currentDeg);
        telemetry.addData("Target Deg", targetDeg);
        telemetry.addData("Error", error);
        telemetry.addData("PID Output", output);
        telemetry.addData("Analog Voltage", analogInput.getVoltage());
        telemetry.update();
    }

    /** Converts analog voltage to absolute degrees [0, 360) */
    private double getAbsolutePosition() {
        return (analogInput.getVoltage() / 3.2 * 360 + offset) % 360;
    }
}
