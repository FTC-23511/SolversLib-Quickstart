package org.firstinspires.ftc.teamcode.Autonomos;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomo rojo lejos", group = "Autonomous")
@Configurable
public class Autonomo_rojo_lejos extends CommandOpMode {
    private Follower follower;

    private final Pose startPose = new Pose(85.3, 6.688, Math.toRadians(90));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2;

    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        waitForStart();

        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        ciclo1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.300, 6.688),
                                new Pose(85.376, 18.285)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.376, 18.285),
                                new Pose(100.348, 34.741)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(100.348, 34.741),
                                new Pose(127.400, 34.846)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(127.400, 34.846),
                                new Pose(85.376, 18.285)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.376, 18.285),
                                new Pose(128.500, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(128.500, 8.894),
                                new Pose(126.800, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(126.800, 8.894),
                                new Pose(128.500, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.500, 8.894),
                                new Pose(85.376, 18.285)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(
                // Ciclo 1 (Score Preload
                new FollowPathCommand(follower, ciclo1, true, 1),
                new WaitCommand(500),
                // Ciclo 2
                new FollowPathCommand(follower, ciclo2, true, 0.85),
                new WaitCommand(500),
                new FollowPathCommand(follower, ciclo2part2, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo3, true, 0.75),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo3parte2, true, 1)

        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}