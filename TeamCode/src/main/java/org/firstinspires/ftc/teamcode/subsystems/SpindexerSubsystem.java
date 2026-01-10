package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.*;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

public class SpindexerSubsystem extends SubsystemBase {

    private final DcMotorEx spindexer;
    private final AnalogInput absoluteEncoder;

    private BallColors[] balls = { NONE, NONE, NONE };

    // PID (tune these)
    private final double kPOriginal = 0.0159;
    private double kP = 0.0159;
    public void updatePIDVoltage(double voltage) {
        double compensation = 13.5 / voltage;
        kP = compensation * kPOriginal;
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

    private double lastAbs = Double.NaN;

    @Override
    public void periodic() {
        double abs = getAbsolutePosition360();

        if (Double.isNaN(lastAbs)) {
            lastAbs = abs;
            continuousPosition = abs;
            return;
        }

        double diff = abs - lastAbs;

        // unwrap based on real analog readings, not modulo continuous position
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;

        continuousPosition += diff;
        lastAbs = abs;

        double outputRaw = pid.calculate(continuousPosition);
        output = clamp(outputRaw, -1, 1);
        spindexer.setPower(output);
    }



    /** Converts analog voltage to 0–360° */
    private double getAbsolutePosition360() {
        return (absoluteEncoder.getVoltage() / 3.2 * 360) % 360;
    }

    /**
     * @return the spindexer's position in unwrapped form (can be negative or positive; not limited to 0 and 360 degrees)
     */
    public double getCurrentPosition() {
        return continuousPosition;
    }

    /**
     * @return the spindexer's position in wrapped form (limited between 0 and 360 degrees)
     */
    public double getWrappedPosition() {
        return getAbsolutePosition360();
    }

    /**
     * @return the spindexer's target position
     */
    public double getPIDSetpoint() {
        return pid.getSetPoint();
    }
    /**
     * @return gets the power output for telemetry
     */
    public double getOutput() {
        return output;
    }
    public void setPIDCoefficients(double p, double i, double d, double f) {
        this.pid.setP(p);
        this.pid.setI(i);
        this.pid.setD(d);
        this.pid.setF(f);
    }

    // Also useful to expose raw voltage for the calibration step
    public double getRawVoltage() {
        return absoluteEncoder.getVoltage();
    }

    /**
     * @return checks how far the spindexer is from target position, returns if it is close enough
     */
    public boolean isNearTargetPosition() { //within 5 deg
        double error = Math.abs(getCurrentPosition() - getPIDSetpoint());
        return error < 5;
    }

    /**
     * @return if the spindexer is moving slowly
     */
    public boolean isLowVelocity() {
        return spindexer.getVelocity() < 20;
    }

    /**
     * @param balls the ball array
     * @return used to clear the third ball after shooting it out the robot
     */
    public void setBalls(BallColors[] balls) {
        this.balls = balls;
    }

    /**
     * @return the array of enums for ball sorting
     */
    public BallColors[] getBalls() {
        return balls;
    }

    /**
     * @param n number of balls
     * @return shifts the array of balls (colors) based on when/if the balls enter/leave the robot
     */
    public void shiftBallsArrayBy(int n) {
        n = ((n % 3) + 3) % 3;
        if (n == 0) return;

        BallColors a = balls[0], b = balls[1], c = balls[2];
        if (n == 1) { balls[0] = b; balls[1] = c; balls[2] = a; }
        else if (n == 2) { balls[0] = c; balls[1] = a; balls[2] = b; }
    }

    /**
     * @param sensor1 color sensor1 result
     * @param sensor2 color sensor2 result
     * @param backSensor back color sensor result
     * @return updates the ball array with specific color if the color sensors detect a ball in that section (index 0 for intake, index 1 for middle, index 2 for shooter)
     */
    public void handleUpdateArray(NormalizedRGBA sensor1, NormalizedRGBA sensor2, NormalizedRGBA backSensor) {
            if (ColorSensorsSubsystem.colorIsGreenIntake(sensor1) || ColorSensorsSubsystem.colorIsGreenIntake(sensor2)) {
                balls[0] = GREEN;
            } else if (ColorSensorsSubsystem.colorIsPurpleIntake(sensor1) || ColorSensorsSubsystem.colorIsPurpleIntake(sensor2)) {
                balls[0] = PURPLE;
            }

            if (ColorSensorsSubsystem.colorIsGreenBack(backSensor)) {
                balls[1] = GREEN;
            } else if (ColorSensorsSubsystem.colorIsPurpleBack(backSensor)) {
                balls[1] = PURPLE;
            }
    }

    /**
     * @param index
     * @param color
     * @return sets the ball color at a specific index, for intervention
     */
    public void setBallAt(int index, BallColors color) {
        balls[index] = color;
    }

}
