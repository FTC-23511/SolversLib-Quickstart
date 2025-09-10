package org.firstinspires.ftc.teamcode.commandbase.subsystems;



import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;

import com.seattlesolvers.solverslib.controller.PIDFController;

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

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakeMotorState.equals(HOLD)) {
            robot.intakeMotor.setPower(INTAKE_HOLD_SPEED);
            Intake.intakeMotorState = intakeMotorState;
        } else if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    robot.intakeMotor.setPower(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                    reverseIntakeTimer.reset();
                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);
                    break;
            }
            Intake.intakeMotorState = intakeMotorState;
        }
    }

    public void toggleActiveIntake(SampleColorTarget sampleColorTarget) {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            if (intakeMotorState.equals(FORWARD)) {
                setActiveIntake(STOP);
            } else if (intakeMotorState.equals(STOP) || intakeMotorState.equals(HOLD)) {
                setActiveIntake(FORWARD);
            }
            Intake.sampleColorTarget = sampleColorTarget;
        }
    }

    public void autoUpdateActiveIntake() {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    if (hasSample()) {
                        if (!readyForColorDetection) {
                            colorDetectionTimer.reset();
                            readyForColorDetection = true;
                        } else if (readyForColorDetection && colorDetectionTimer.milliseconds() > 50) {
                            readyForColorDetection = false;

                            sampleColor = sampleColorDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                            if (correctSampleDetected()) {
                                setActiveIntake(HOLD);
                                if (opModeType.equals(OpModeType.TELEOP)) {
                                    if (sampleColorTarget.equals(ANY_COLOR)) {
                                        if (soloTeleOp) {
                                            new SequentialCommandGroup(
                                                    new RealTransfer(robot).beforeStarting(new WaitCommand(250))
//                                                    new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false)
                                            ).schedule(false);
                                        } else {
                                            new RealTransfer(robot).beforeStarting(
                                                    new WaitCommand(250)
                                            ).schedule(false);
                                        }
                                    } else {
                                        new SetIntake(robot, TRANSFER, HOLD, 0, false).schedule(false);
                                    }
                                }
                            } else {
                                reverseIntakeTimer.reset();
                                setActiveIntake(REVERSE);
                            }
                        }
                    }
                    break;
                case REVERSE:
                    if (!hasSample() && !waitingForReverse) {
                        reverseIntakeTimer.reset();
                        waitingForReverse = true;
                    } else if (!hasSample() && waitingForReverse && reverseIntakeTimer.milliseconds() > REVERSE_TIME_MS) {
                        waitingForReverse = false;
                        if (opModeType.equals(OpModeType.TELEOP)) {
                            setActiveIntake(FORWARD);
                        } else {
                            setActiveIntake(STOP);
                        }
                    }
                    break;
                case HOLD:
                    if (!correctSampleDetected() && hasSample() && Intake.intakePivotState.equals(INTAKE)) {
                        setActiveIntake(REVERSE);
                    }
                    break;
                // No point of setting intakeMotor to 0 again
            }
        } else if (intakePivotState.equals(TRANSFER) || intakePivotState.equals(INSIDE)) {
            setActiveIntake(HOLD);
        }
    }

    public static SampleColorDetected sampleColorDetected(int red, int green, int blue) {
        if (blue >= green && blue >= red) {
            return BLUE;
        } else if (green >= red) {
            return YELLOW;
        } else {
            return RED;
        }
    }

    public static boolean correctSampleDetected() {
        switch (sampleColorTarget) {
            case ANY_COLOR:
                if (sampleColor.equals(YELLOW) ||
                        (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                                (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED)))) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                        (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))) {
                    return true;
                }
                break;
        }
        return false;
    }
    public boolean hasSample() {
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
        autoUpdateExtendo();
        autoUpdateActiveIntake();
    }
}
