package org.firstinspires.ftc.teamcode.commandbase.subsystems;



import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake.IntakeMotorState.*;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.globals.Constants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum IntakeMotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public enum IntakePivotState {
        INTAKE,
        TRANSFER,
        INTAKE_READY,
        HOLD
    }

    public static IntakeMotorState intakeMotorState = IntakeMotorState.STOP;
    public static IntakePivotState intakePivotState = IntakePivotState.HOLD;

    public void init() {
        setPivot(IntakePivotState.INTAKE);
        robot.colorSensor.enableLed(true);
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch (intakePivotState) {
            case TRANSFER:
                robot.leftIntakePivot.set(INTAKE_PIVOT_TRANSFER_POS);
                robot.rightIntakePivot.set(INTAKE_PIVOT_TRANSFER_POS);
                break;

            case INTAKE:
                robot.leftIntakePivot.set(INTAKE_PIVOT_INTAKE_POS);
                robot.rightIntakePivot.set(INTAKE_PIVOT_INTAKE_POS);
                break;

            case INTAKE_READY:
                robot.leftIntakePivot.set(INTAKE_PIVOT_READY_INTAKE_POS);
                robot.rightIntakePivot.set(INTAKE_PIVOT_READY_INTAKE_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setIntake(IntakeMotorState intakeMotorState) {
        if (intakeMotorState.equals(IntakeMotorState.HOLD)) {
            robot.intakeMotor.set(INTAKE_HOLD_SPEED);
            Intake.intakeMotorState = intakeMotorState;
        } else if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
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
            Intake.intakeMotorState = intakeMotorState;
        }
    }

    public void toggleIntake() {
        if (intakePivotState.equals(IntakePivotState.INTAKE) || intakePivotState.equals(IntakePivotState.INTAKE_READY)) {
            if (intakeMotorState.equals(IntakeMotorState.FORWARD)) {
                setIntake(IntakeMotorState.STOP);
            } else if (intakeMotorState.equals(IntakeMotorState.STOP) || intakeMotorState.equals(IntakeMotorState.HOLD)) {
                setIntake(FORWARD);
            }
        }
    }

    public void updateIntake() {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
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
        } else if (intakePivotState.equals(TRANSFER) || intakePivotState.equals(IntakePivotState.HOLD)) {
            setIntake(IntakeMotorState.HOLD);
        }
    }

    public boolean hasArtifact() {
        /* Color thresholding (not used)
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        SampleColorDetected sampleColor = sampleColorDetected(red, green, blue);

        switch (sampleColor) {
            case YELLOW:
                if (green > YELLOW_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case RED:
                if (red > RED_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case BLUE:
                if (blue > BLUE_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
        }
         */

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        return distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD;
    }

    @Override
    public void periodic() {
        updateIntake();
    }
}
