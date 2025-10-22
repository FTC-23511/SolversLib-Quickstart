package org.firstinspires.ftc.teamcode.samples;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.HoldPointCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class PedroCommands extends CommandOpMode {
    Follower follower;

    Pose pose = new Pose(
            72, 72, 90
    );

    PathChain pathChain;

    @Override
    public void initialize() {
        super.reset();

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, Math.toRadians(0)),
                        new Pose(16, 28, Math.toRadians(90)))
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        schedule(
                // Updates follower to follow path
                new RunCommand(() -> follower.update()),

                // HoldPointCommand
                new HoldPointCommand(follower, new Pose(0, 4, 0), false),
                new HoldPointCommand(follower, pose, true),

                // TurnCommand
                new TurnCommand(follower, Math.PI / 2, false),
                new TurnCommand(follower, 90.0, true, AngleUnit.DEGREES),

                // TurnToCommand
                new TurnToCommand(follower, Math.PI / 2),
                new TurnToCommand(follower, 90.0, AngleUnit.DEGREES),

                // FollowPathCommand
                new FollowPathCommand(follower, pathChain),
                new FollowPathCommand(follower, pathChain, true),
                new FollowPathCommand(follower, pathChain, true, 1.0),
                new FollowPathCommand(follower, pathChain, true, 1.0).setGlobalMaxPower(1.0)
        );
    }

    @Override
    public void run() {
        super.run();
    }
}