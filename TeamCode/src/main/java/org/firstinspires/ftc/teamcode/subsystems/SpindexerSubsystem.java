package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class SpindexerSubsystem extends SubsystemBase {
    private final DcMotor spindexer;

    // PIDF Coefficients
    private double kP = -0.0004;
    private final double kI = 0.000000;
    private final double kD = 0.000001;
    private final double kF = 0.000;

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int currentPosition = 0;


    // Target position for PIDF 
    private double targetPosition = 0;

    // PIDF Controller
    private final PIDController pid;

    public enum SpindexerState { ONE, TWO, THREE }

    public SpindexerState getSpindexerState() {
        return spindexerState;
    }

    public SpindexerState spindexerState = SpindexerState.ONE;

    private double lastOutput = -9999999;
    private double output = 0;

    public SpindexerSubsystem(final HardwareMap hm) {
        spindexer = hm.get(DcMotor.class, "spindexer");
        pid = new PIDController(kP, kI, kD);
        pid.setPID(kP, kI, kD);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void advanceSpindexer() {
        targetPosition += SPINDEXER_TICKS_PER_DEG * 120;

        switch (spindexerState) {
            case ONE:   spindexerState = SpindexerState.TWO;   break;
            case TWO:   spindexerState = SpindexerState.THREE; break;
            case THREE: spindexerState = SpindexerState.ONE;   break;
        }
    }
    public void reverseSpindexer() {
        targetPosition -= SPINDEXER_TICKS_PER_DEG * 120;

        switch (spindexerState) {
            case ONE:   spindexerState = SpindexerState.THREE;   break;
            case TWO:   spindexerState = SpindexerState.ONE; break;
            case THREE: spindexerState = SpindexerState.TWO;   break;
        }
    }
    public void moveSpindexerBy(double x) {
        targetPosition += x;
    }

    @Override
    public void periodic() {

//        currentPosition = spindexer.getCurrentPosition();
//        output = pid.calculate(currentPosition, targetPosition);
//        double clipped = clamp(output, -CLAMP_LIMIT, CLAMP_LIMIT);
////
//        if (Math.abs(lastOutput - clipped) > SPINDEXER_CACHETHRESHOLD) {
//            spindexer.setPower(clipped);
//            lastOutput = output;
//        }
        pid.setP(kP);
        currentPosition = spindexer.getCurrentPosition();
        output = pid.calculate(currentPosition, targetPosition);
        spindexer.setPower(clamp(output, -0.4, 1));
    }

    // Get current PID output
    public String getOutput() {
        return output + " | " + clamp(output, -0.5, 1);
    }


    // Return current setpoint (your tracked target position)
    public double getPIDSetpoint() {
        return targetPosition;
    }

    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 12) * -0.0004;
    }
}
