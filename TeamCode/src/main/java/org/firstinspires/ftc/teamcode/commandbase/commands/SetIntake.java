package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class SetIntake extends CommandBase {
    private final Robot robot;
    private final Intake.MotorState motorState;
    private final Intake.PivotState pivotState;
    private boolean waitForArtifacts;

    private ElapsedTime timer;

    public SetIntake(Robot robot, Intake.MotorState motorState, Intake.PivotState pivotState) {
        this(robot, motorState, pivotState, false);
    }

    public SetIntake(Robot robot, Intake.MotorState motorState, Intake.PivotState pivotState, boolean waitForArtifacts) {
        this.robot = robot;
        this.motorState = motorState;
        this.pivotState = pivotState;
        this.waitForArtifacts = waitForArtifacts;

        timer = new ElapsedTime();

        robot = Robot.getInstance();
        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.intake.setIntake(motorState);
        robot.intake.setPivot(pivotState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (waitForArtifacts && robot.intake.transferFull()) {
            return true;
        }

        return !waitForArtifacts && timer.milliseconds() > 200;
    }
}
