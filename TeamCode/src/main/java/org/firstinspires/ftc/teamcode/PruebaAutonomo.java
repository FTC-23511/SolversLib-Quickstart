package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Solvers Auto", group = "Autonomous")
public class PruebaAutonomo extends CommandOpMode {

    private Follower follower;
    private TelemetryData telemetryData = new TelemetryData(telemetry);

    // Poses principales
    private final Pose startPose = new Pose(30.631, 132.441, Math.toRadians(0));

    // PathChains extraídas de tu código
    private PathChain ciclo1, ciclo2, ciclo3, ciclo4, ciclo5;

    // ==================== MECANISMOS ====================
    // Reemplaza estos con tus comandos reales de subsistemas



    private void buildPaths() {
        follower.setStartingPose(startPose);

        // Usando exactamente las trayectorias que me pasaste
        ciclo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(30.631, 132.441), new Pose(52.051, 83.325)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.051, 83.325),
                        new Pose(40.567, 48.326),
                        new Pose(30.508, 59.887),
                        new Pose(22.353, 61.323),
                        new Pose(9.714, 54.911)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Pose(9.714, 54.911),
                        new Pose(49.440, 51.685),
                        new Pose(52.051, 83.325)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.051, 83.325),
                        new Pose(37.135, 60.360),
                        new Pose(12.469, 58.513)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(153))
                .addPath(new BezierLine(new Pose(12.469, 58.513), new Pose(11.272, 58.218)))
                .setLinearHeadingInterpolation(Math.toRadians(153), Math.toRadians(145))
                .addPath(new BezierCurve(
                        new Pose(11.272, 58.218),
                        new Pose(48.907, 64.523),
                        new Pose(52.051, 83.325)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();

        ciclo4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.051, 83.325),
                        new Pose(37.135, 60.360),
                        new Pose(11.272, 58.218)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .addPath(new BezierCurve(
                        new Pose(11.272, 58.218),
                        new Pose(48.907, 64.523),
                        new Pose(52.051, 83.325)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();

        ciclo5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(52.051, 83.325),
                        new Pose(37.135, 60.360),
                        new Pose(11.272, 58.218)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .addPath(new BezierCurve(
                        new Pose(11.272, 58.218),
                        new Pose(48.907, 64.523),
                        new Pose(52.051, 83.325)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(145))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        schedule(
                // Ciclo 1
                new FollowPathCommand(follower, ciclo1),
                new WaitCommand(1500),

                // Ciclo 2
                new FollowPathCommand(follower, ciclo2),
                new WaitCommand(500),
                new FollowPathCommand(follower, ciclo3), // regreso a scoring position
                new WaitCommand(600),

                // Ciclo 3
                new FollowPathCommand(follower, ciclo4),
                new WaitCommand(500),
                new FollowPathCommand(follower, ciclo5)
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.addData("Path State", "Running");
        telemetryData.update();
    }
}