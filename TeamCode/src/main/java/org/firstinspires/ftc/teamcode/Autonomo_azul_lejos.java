package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomo azul lejos", group = "Autonomous")
@Configurable
public class Autonomo_azul_lejos extends CommandOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;

    private final Pose startPose = new Pose(56, 6.688, Math.toRadians(90));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

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
                                new Pose(56.000, 6.688),
                                new Pose(56.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 18.723),
                                new Pose(45.235, 34.959)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(45.235, 34.959),
                                new Pose(13.676, 35.284)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.676, 35.284),
                                new Pose(56.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 18.723),
                                new Pose(15.5, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(15.5, 8.894),
                                new Pose(17.200, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(17.200, 8.894),
                                new Pose(15.5, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.5, 8.894),
                                new Pose(56.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(
                // Ciclo 1 (Score Preload)
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
        panelsTelemetry.update(telemetry);
    }
}