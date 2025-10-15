package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class SpindexerSubsystem extends SubsystemBase {
    private final DcMotor spindexer;

    // PIDF Coefficients
    private final double kP = -0.0005;
    private final double kI = 0.000002;
    private final double kD = 0.000001;
    private final double kF = 0.000;

    public double getCurrentPosition() {
        return currentPosition;
    }

    private double currentPosition = 0;

    private static final double CLAMP_LIMIT = 0.4;

    // Target position for PIDF
    private double targetPosition = SPINDEXER_INITPOS;

    // PIDF Controller
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    public enum SpindexerState { ONE, TWO, THREE }
    public SpindexerState spindexerState = SpindexerState.ONE;

    private double lastOutput = -9999999;
    private double output = 0;

    public SpindexerSubsystem(final HardwareMap hm) {
        spindexer = hm.get(DcMotor.class, "spindexer");
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

    @Override
    public void periodic() {
        lastOutput = output;
        currentPosition = spindexer.getCurrentPosition();
        output = pidf.calculate(currentPosition, targetPosition);
//
//        if (Math.abs(lastOutput - output) > SPINDEXER_CACHETHRESHOLD) {
            spindexer.setPower(clamp(output, -CLAMP_LIMIT, CLAMP_LIMIT));
//        }
    }

    // Get current PID output
    public double getOutput() {
        return output;
    }


    // Return current setpoint (your tracked target position)
    public double getPIDSetpoint() {
        return targetPosition;
    }

    //kinda useless
    public void killSpindexerPower() {
        spindexer.setPower(0.0);
    }
}
