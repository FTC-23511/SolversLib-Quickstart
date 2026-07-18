package org.firstinspires.ftc.teamcode.Autonomos;
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
import Subsistemas.IntakeSubsystem_Autonomous;
import Subsistemas.TurretSubsystem_Autonomous;

@Autonomous(name = "Autonomo azul cerca", group = "Autonomous")
@Configurable
public class Autonomo_azul_cerca extends CommandOpMode {
    private Follower follower;
    private IntakeSubsystem_Autonomous intake;
    private TurretSubsystem_Autonomous turret;

    private final Pose startPose = new Pose(38.285, 134.628, Math.toRadians(-90));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2, Ciclo4,Ciclo4parte2,Ciclo5,CIclo5part2;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new IntakeSubsystem_Autonomous(hardwareMap);
        turret = new TurretSubsystem_Autonomous(hardwareMap);

        buildPaths();
        waitForStart();

        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        ciclo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(38.285, 134.628), new Pose(56.425, 82.888)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.425, 82.888),new Pose(45.725, 58.763)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierLine(new Pose(45.725, 58.763),new Pose(21.076, 58.702)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(21.076, 58.702), new Pose(48.936, 60.047), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(56.425, 82.888), new Pose(37.253, 55.456), new Pose(12.704, 58.332)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(12.704, 58.332), new Pose(44.406, 62.236), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        Ciclo4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(56.425, 82.888), new Pose(37.253, 55.456), new Pose(12.704, 58.332)))
                .setConstantHeadingInterpolation(Math.toRadians(144))
                .build();

        Ciclo4parte2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(12.704, 58.332), new Pose(44.406, 62.236), new Pose(56.425, 82.888)))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        Ciclo5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.425, 82.888), new Pose(15.653, 82.522)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        CIclo5part2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15.653, 82.522), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(
                // Ciclo 1 (Score Preload)
                new FollowPathCommand(follower, ciclo1, true, 1),
                new WaitCommand(500),
                // Ciclo 2
                new FollowPathCommand(follower, ciclo2, true, 0.9),
                new WaitCommand(500),
                new FollowPathCommand(follower, ciclo2part2, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo3, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo3parte2, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo4, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo4parte2, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, Ciclo5, true, 1),
                new WaitCommand(500),
                new FollowPathCommand(follower, CIclo5part2, true, 1)
        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}