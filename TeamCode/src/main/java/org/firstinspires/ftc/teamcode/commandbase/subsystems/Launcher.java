package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake.IntakePivotState.INTAKE;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake.IntakePivotState.INTAKE_READY;
import static org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake.IntakePivotState.TRANSFER;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.globals.Constants.INTAKE_REVERSE_SPEED;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class Launcher extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum LauncherMotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public static LauncherMotorState launcherMotorState = LauncherMotorState.STOP;

    public void init() {

    }

    public void setLauncher(LauncherMotorState launcherMotorState) {
        if (launcherMotorState.equals(LauncherMotorState.HOLD)) {
            robot.intakeMotor.set(INTAKE_HOLD_SPEED);
        } else {
            switch (launcherMotorState) {
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
        Launcher.launcherMotorState = launcherMotorState;
    }

    public void updateLauncher() {
        // TODO: Add this
    }

    @Override
    public void periodic() {
        updateLauncher();
    }
}
