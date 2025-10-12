package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_REVERSE_SPEED;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum MotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public static MotorState motorState = MotorState.STOP;

    public void init() {

    }

    public void setLauncher(MotorState motorState) {
        if (motorState.equals(MotorState.HOLD)) {
            robot.intakeMotor.set(INTAKE_HOLD_SPEED);
        } else {
            switch (motorState) {
                case FORWARD:
                    robot.launchMotors.set(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.launchMotors.set(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.launchMotors.set(0);
                    break;
            }
        }
        Turret.motorState = motorState;
    }

    public void updateLauncher() {
        // TODO: Add this
    }

    @Override
    public void periodic() {
        updateLauncher();
    }
}
