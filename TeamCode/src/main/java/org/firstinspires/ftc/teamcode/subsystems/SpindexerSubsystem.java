package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

public class SpindexerSubsystem extends SubsystemBase {
    /*
        Info:
        The spindexer state can be 1-3. When the spindexer motor moves, the state also increases
        The balls in the spindexer are 1 2 and 3 starting from the intake and ending at the shooter
     */
    private final DcMotorEx spindexer;

    //Store what balls are in the spindexer
    public BallColors[] balls = {NONE, NONE, NONE};
    //Store what state the spindexer is in
//    public enum SpindexerState {ONE, TWO, THREE} //unused i think??
//    public SpindexerState spindexerState = SpindexerState.ONE;
//    public SpindexerState getSpindexerState() {return spindexerState;}



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


    private double lastOutput = -9999999;
    private double output = 0;

    public SpindexerSubsystem(final HardwareMap hm) {
        spindexer = hm.get(DcMotorEx.class, "spindexer");
        pid = new PIDController(kP, kI, kD);
        pid.setPID(kP, kI, kD);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveSpindexerBy(double x) {
        targetPosition += x;
    }

    @Override
    public void periodic() {
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
        kP = (voltage / 13.5) * -0.0004;
    }
    //@return boolean if spindexer is not moving and at a target position.

    public boolean checkSpindexerPosition() {
        return (Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition()) < (SPINDEXER_TICKS_PER_DEG * 5));
    }

    public boolean checkSpindexerVelocity() {
        return spindexer.getVelocity() < (SPINDEXER_TICKS_PER_DEG * 120)/5.0;
    }

    public boolean availableToSenseColor() {
        if (checkSpindexerPosition() && checkSpindexerVelocity()) {
            return true;
        }
        else {
            return false;
        }
    }
    public void setBalls(BallColors[] balls) {
        this.balls = balls;
    }
    public BallColors[] getBalls() {
        return balls;
    }
    //forward = spindexer forward, vice versa
    public void shiftBallsArrayBy(int n) {
        n = ((n % 3) + 3) % 3; // normalize n to 0, 1, or 2
        if (n == 0) return;

        BallColors a = balls[0], b = balls[1], c = balls[2];

        if (n == 1) {
            // [0,5,2] -> [5,2,0]
            balls[0] = b;
            balls[1] = c;
            balls[2] = a;
        } else if (n == 2) {
            // [0,5,2] -> [2,0,5]
            balls[0] = c;
            balls[1] = a;
            balls[2] = b;
        }
    }
    public void setBallAt(int index, BallColors color) {
        balls[index] = color;
    }
}
