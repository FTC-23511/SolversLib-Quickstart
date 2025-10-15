package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.globals.Robot;

public class SetIntake extends CommandBase {
    private final Robot robot;
    private final Intake.MotorState motorState;
    private final Intake.PivotState pivotState;

    public SetIntake(Robot robot, Intake.MotorState motorState, Intake.PivotState pivotState) {
        this.robot = robot;
        this.motorState = motorState;
        this.pivotState = pivotState;

        ElapsedTime timer;

        robot = Robot.getInstance();
        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true; // TODO: replace with end condition of the command
    }
}
