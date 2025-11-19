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

    private BallColors[] balls = {NONE, NONE, NONE};

    // PID coefficients (tuned for degrees)
    private double kP = -0.0159;
    private final double kI = 0;
    private final double kD = 0;

    private double currentPositionDeg = 0;
    private double targetPositionDeg = 0;
    private double offset = 340; // in degrees

    private final PIDController pid;
    private double output = 0;

    public SpindexerSubsystem(HardwareMap hm) {
        spindexer = hm.get(DcMotorEx.class, "spindexer");
        absoluteEncoder = hm.get(AnalogInput.class, "spindexerAnalog");

        pid = new PIDController(kP, kI, kD);
        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize target to current position
        currentPositionDeg = getAbsolutePosition();
        targetPositionDeg = currentPositionDeg;
    }

    /** Move spindexer by degrees (absolute control) */
    public void moveSpindexerBy(double degrees) {
        targetPositionDeg = (targetPositionDeg + degrees) % 360;
        if (targetPositionDeg < 0) targetPositionDeg += 360;
    }

    public void set(double degrees) {
        targetPositionDeg = degrees;
    }

    @Override
    public void periodic() {
        currentPositionDeg = getAbsolutePosition();

        // Compute shortest angular difference [-180, 180]
        double error = targetPositionDeg - currentPositionDeg;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        // PID using wrapped error
        output = pid.calculate(0, error);  // or use a custom simple P controller
        output = clamp(output, -1, 1);

        spindexer.setPower(output);
    }


    /** Converts analog voltage to absolute position in degrees [0,360) */
    private double getAbsolutePosition() {
        return (absoluteEncoder.getVoltage() / 3.2 * 360 + offset) % 360;
    }

    public double getCurrentPosition() {
        return currentPositionDeg;
    }

    public double getPIDSetpoint() {
        return targetPositionDeg;
    }

    public double getOutput() {
        return output;
    }

    public boolean isNearTargetPosition() {
        return Math.abs(((targetPositionDeg - currentPositionDeg + 180) % 360 + 360) % 360 - 180) < 5; // 5° tolerance
    }

    public boolean isLowVelocity() {
        return spindexer.getVelocity() < 20; // tune as needed
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
        if (n == 1) { balls[0]=b; balls[1]=c; balls[2]=a; }
        else if (n == 2) { balls[0]=c; balls[1]=a; balls[2]=b; }
    }

    public void setBallAt(int index, BallColors color) {
        balls[index] = color;
    }
    public void updatePIDVoltage(double voltage) {
        kP = (voltage / 13.5) * -0.0159;
    }


}
