package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        TRANSFER
    }

    public enum PivotState {
        INTAKE,
        TRANSFER,
        HOLD
    }

    public static MotorState motorState = MotorState.STOP;
    public static PivotState pivotState = PivotState.HOLD;

    public void init() {
        setPivot(PivotState.INTAKE);
    }

    public void setPivot(PivotState pivotState) {
        switch (pivotState) {
            case HOLD:
                robot.intakePivotServo.set(INTAKE_PIVOT_HOLD);
                break;
            case TRANSFER:
                robot.intakePivotServo.set(INTAKE_PIVOT_TRANSFER);
                break;
            case INTAKE:
                robot.intakePivotServo.set(INTAKE_PIVOT_INTAKE);
                break;
        }

        Intake.pivotState = pivotState;
    }

    public void setIntake(MotorState motorState) {
        switch (motorState) {
            case STOP:
                robot.intakeMotor.set(0);
                break;
            case TRANSFER:
                robot.intakeMotor.set(INTAKE_TRANSFER_SPEED);
                break;
            case FORWARD:
                robot.intakeMotor.set(INTAKE_FORWARD_SPEED);
                break;
            case REVERSE:
                robot.intakeMotor.set(INTAKE_REVERSE_SPEED);
                break;
        }
    }

    public void toggleIntake() {
        if (pivotState.equals(PivotState.INTAKE)) {
            if (motorState.equals(MotorState.FORWARD)) {
                setIntake(MotorState.STOP);
            } else if (motorState.equals(MotorState.STOP)) {
                setIntake(MotorState.FORWARD);
            }
        }
    }

    public void updateIntake() {
        if (pivotState.equals(PivotState.INTAKE)) {
            switch (motorState) {
                case FORWARD:
                    if (hasArtifact()) {
                        // TODO: add code for forward
                    }
                    break;
                case REVERSE:
                    // ..
                    break;
                // No point of setting intakeMotor to 0 again
            }
        } else if (pivotState.equals(PivotState.HOLD)) {
            setIntake(MotorState.STOP);
        }
    }

    public boolean hasArtifact() {
        return robot.distanceSensor.targetReached(robot.distanceTarget);
    }

    @Override
    public void periodic() {
        updateIntake();
    }
}
