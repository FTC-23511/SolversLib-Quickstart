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

    private double output = 0;

    private double continuousPosition = 0;
    private boolean initialized = false;

    public SpindexerSubsystem(HardwareMap hm) {
        spindexer = hm.get(DcMotorEx.class, "spindexer");
        absoluteEncoder = hm.get(AnalogInput.class, "spindexerAnalog");

        pid = new PIDController(kP, kI, kD);

        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** Moves spindexer by a number of degrees while keeping direction consistent */
    public void moveSpindexerBy(double deltaDegrees) {
        pid.setSetPoint(pid.getSetPoint() - deltaDegrees);
    }



    /** Set absolute target (but still unbounded) */
    public void set(double degrees) {
        pid.setSetPoint(degrees);
    }

    @Override
    public void periodic() {
        double abs = getAbsolutePosition360();

        if (!initialized) {
            continuousPosition = abs;
            initialized = true;
        } else {
            double last = continuousPosition % 360;
            double diff = abs - last;

            // wrap diff into [-180, 180]
            diff = ((diff + 180) % 360 + 360) % 360 - 180;

            continuousPosition += diff;
        }

        double outputRaw = pid.calculate(continuousPosition);
        output = clamp(outputRaw, -1, 1);

        spindexer.setPower(output);
    }


    /** Converts analog voltage to 0–360° */
    private double getAbsolutePosition360() {
        return (absoluteEncoder.getVoltage() / 3.2 * 360) % 360;
    }

    public double getCurrentPosition() {
        return continuousPosition;
    }

    public double getWrappedPosition() {
        return getAbsolutePosition360();
    }

    public double getPIDSetpoint() {
        return pid.getSetPoint();
    }
    public double getOutput() {
        return output;
    }

    public boolean isNearTargetPosition() { //within 5 deg
        double error = Math.abs(getCurrentPosition() - getPIDSetpoint());
        return error < 5;
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
