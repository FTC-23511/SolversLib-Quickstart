package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.globals.Robot;

public class DriveTo extends CommandBase {
    private final Robot robot;
    private final Pose2d target;

    public DriveTo(Pose2d pose) {
        target = pose;
        robot = Robot.getInstance();
        addRequirements(robot.drive);
    }

    @Override
    public void initialize() {
        robot.drive.follower.setTarget(target);
    }

    @Override
    public void execute() {
        robot.drive.swerve.updateWithTargetVelocity(robot.drive.follower.calculate(robot.drive.getPose()));
    }

    @Override
    public boolean isFinished() {
        return robot.drive.follower.atTarget();
    }
}
