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
        HOLD
    }

    public enum PivotState {
        INTAKE,
        TRANSFER,
        INTAKE_READY,
        HOLD
    }

    public static MotorState motorState = MotorState.STOP;
    public static PivotState pivotState = PivotState.HOLD;

    public void init() {
        setPivot(PivotState.INTAKE);
    }

    public void setPivot(PivotState pivotState) {
        switch (pivotState) {
            case TRANSFER:
                robot.intakePivotServo.set(INTAKE_PIVOT_TRANSFER);
                break;

            case INTAKE:
                robot.intakePivotServo.set(INTAKE_PIVOT_INTAKE);
                break;

            case INTAKE_READY:
                robot.intakePivotServo.set(INTAKE_PIVOT_READY_INTAKE);
                break;
        }

        Intake.pivotState = pivotState;
    }

    public void setIntake(MotorState motorState) {
        if (motorState.equals(MotorState.HOLD)) {
            robot.intakeMotor.set(INTAKE_HOLD_SPEED);
            Intake.motorState = motorState;
        } else if (pivotState.equals(PivotState.INTAKE) || pivotState.equals(PivotState.INTAKE_READY)) {
            switch (motorState) {
                case FORWARD:
                    robot.intakeMotor.set(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.set(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.intakeMotor.set(0);
                    break;
            }
            Intake.motorState = motorState;
        }
    }

    public void toggleIntake() {
        if (pivotState.equals(PivotState.INTAKE) || pivotState.equals(PivotState.INTAKE_READY)) {
            if (motorState.equals(MotorState.FORWARD)) {
                setIntake(MotorState.STOP);
            } else if (motorState.equals(MotorState.STOP) || motorState.equals(MotorState.HOLD)) {
                setIntake(MotorState.FORWARD);
            }
        }
    }

    public void updateIntake() {
        if (pivotState.equals(PivotState.INTAKE) || pivotState.equals(PivotState.INTAKE_READY)) {
            switch (motorState) {
                case FORWARD:
                    if (hasArtifact()) {
                        // TODO: add code for forward
                    }
                    break;
                case REVERSE:
                    // ..
                    break;
                case HOLD:
                    // ...
                    break;
                // No point of setting intakeMotor to 0 again
            }
        } else if (pivotState.equals(PivotState.TRANSFER) || pivotState.equals(PivotState.HOLD)) {
            setIntake(MotorState.HOLD);
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
