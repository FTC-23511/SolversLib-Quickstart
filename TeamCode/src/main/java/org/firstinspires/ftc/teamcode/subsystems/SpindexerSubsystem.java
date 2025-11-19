package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

public class SpindexerSubsystem extends SubsystemBase {

    private final DcMotorEx spindexer;
    private final AnalogInput absoluteEncoder;

    private BallColors[] balls = { NONE, NONE, NONE };

    // PID (tune these)
    private double kP = 0.0159;
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 13.5) * 0.0159;
    }
    private double kI = 0;
    private double kD = 0.0000114;

    private final PIDController pid;

    // Absolute values
    private double currentDeg = 0;         // unwrapped actual angle
    private double lastWrappedDeg = 0;     // internal tracker
    public double offset = 0;

    // IMPORTANT: unbounded target
    private double targetDeg = 0;

    private double output = 0;

    public SpindexerSubsystem(HardwareMap hm) {
        spindexer = hm.get(DcMotorEx.class, "spindexer");
        absoluteEncoder = hm.get(AnalogInput.class, "spindexerAnalog");

        pid = new PIDController(kP, kI, kD);

        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize starting position
        double initial = getAbsolutePosition360();
        lastWrappedDeg = initial;
        currentDeg = initial;
        targetDeg = initial;
    }

    /** Moves spindexer by a number of degrees while keeping direction consistent */
    public void moveSpindexerBy(double deltaDegrees) {
        targetDeg += deltaDegrees;  // unbounded — preserves direction!
    }

    /** Set absolute target (but still unbounded) */
    public void set(double degrees) {
        double current = currentDeg;
        double wrappedTarget = degrees % 360;

        // Compute shortest path
        double diff = wrappedTarget - (current % 360);
        diff = ((diff + 180) % 360 + 360) % 360 - 180;

        targetDeg = current + diff;
    }

    @Override
    public void periodic() {

        // Step 1: read 0–360°
        double wrapped = getAbsolutePosition360();

        // Step 2: unwrap into continuous range
        double base = (lastWrappedDeg % 360 + 360) % 360;
        double delta = wrapped - base;

        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        lastWrappedDeg += delta;
        currentDeg = lastWrappedDeg;

        // Step 3: PD control on continuous error
        double error = targetDeg - currentDeg;
        output = pid.calculate(0, error);

        // Clamp power
        output = clamp(output, -1, 1);


        spindexer.setPower(output);
    }

    /** Converts analog voltage to 0–360° */
    private double getAbsolutePosition360() {
        return (absoluteEncoder.getVoltage() / 3.2 * 360 + offset) % 360;
    }

    public double getCurrentPosition() {
        return currentDeg;
    }
    public double getPIDSetpoint() {
        return targetDeg;
    }
    public double getOutput() {
        return output;
    }

    public boolean isNearTargetPosition() {
        return Math.abs(targetDeg - currentDeg) < 5;
    }

    public boolean isLowVelocity() {
        return spindexer.getVelocity() < 20;
    }

    public boolean availableToSenseColor() {
        return isNearTargetPosition() && isLowVelocity();
    }

    public void setBalls(BallColors[] balls) {
        this.balls = balls;
    }

    public BallColors[] getBalls() {
        return balls;
    }

    public void shiftBallsArrayBy(int n) {
        n = ((n % 3) + 3) % 3;
        if (n == 0) return;

        BallColors a = balls[0], b = balls[1], c = balls[2];
        if (n == 1) { balls[0] = b; balls[1] = c; balls[2] = a; }
        else if (n == 2) { balls[0] = c; balls[1] = a; balls[2] = b; }
    }

    public void setBallAt(int index, BallColors color) {
        balls[index] = color;
    }

}
