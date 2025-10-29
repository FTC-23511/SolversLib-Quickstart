package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

@Config
@Autonomous(name = "Red🦅", group = "angryBirds")
public class RedAuto extends CommandOpMode {
    //paths
    /*
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
     */


    private final ArrayList<PathChain> paths = new ArrayList<>();

    //stuff

    private ElapsedTime timer;

    //private final ArrayList<PathChain> paths = new ArrayList<>();

    //private DashboardPoseTracker dashboardPoseTracker; they had this in github code and I thought it might be useful later

    //subsytems and pedro

    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(123.36079077429983,122.17462932454696,45); //find actual statring pos
    private IntakeSubsystem intake;
    private ShooterSubSystem shooter;
    private SpindexerSubsystem spindexer;
    private LEDSubSystem led;


    public void buildPaths(Follower follower) {
        follower.setStartingPose(startingPose);
        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.361, 122.175), new Pose(84.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(90, 80), new Pose(101.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(101.000, 84.000), new Pose(130.000, 84.000))
                )
                .setTangentHeadingInterpolation()
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 84.000), new Pose(84.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(90,  80), new Pose(101.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(101.000, 60.000), new Pose(135.000, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(135.000, 60.000), new Pose(125.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 60.000), new Pose(84.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(90, 80), new Pose(84.000, 108.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build()
        );

    }
    //preset command methods
    public SequentialCommandGroup shootArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new WaitCommand(1500),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new WaitCommand(1500),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new WaitCommand(1500)
        );
    }

    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)),
                new WaitCommand(2000),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new WaitCommand(2000),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new WaitCommand(2000),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.STILL))
        );
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubSystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        led = new LEDSubSystem(hardwareMap);

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter);

        //init paths
        buildPaths(follower);


        //schedule commands
        //one cycle = 3 balls + shoot given starting position is right at the shooting spot
        //pp file is editable but you have to update the buildPath

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> follower.update()),

                new SequentialCommandGroup(
                        new InstantCommand(() -> {shooter.setTargetVelocity(1300);}), //start shoot
                        new FollowPathCommand(follower, paths.get(0)), //drive to shooting pos
                        new WaitCommand(1500),
                        shootArtifacts(),

                        //cycle one
                        new FollowPathCommand(follower, paths.get(1)),
                        new ParallelCommandGroup(
                                intakeArtifacts(),
                                new FollowPathCommand(follower, paths.get(2), 0.2)
                        ),
                        new FollowPathCommand(follower, paths.get(3), true),
                        //needs time for shooter to ramp up
                        new WaitCommand(1500),
                        shootArtifacts(),

                        new WaitCommand(1000),

                        //cycle two
                        new FollowPathCommand(follower, paths.get(4)),
                        new ParallelCommandGroup(
                                intakeArtifacts(),
                                new FollowPathCommand(follower, paths.get(5), true, 0.2)
                        ),
                        //needs extra step to back out from the wall because it will collide with the exit of the ramp
                        new FollowPathCommand(follower, paths.get(6)),

                        new FollowPathCommand(follower, paths.get(7), true),
                        new WaitCommand(1500),
                        shootArtifacts(),

                        //move off shooting line so that you get extra points theoretically
                        new FollowPathCommand(follower, paths.get(8))
                )
        );


    }
    @Override
    public void run() {
        if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -50) {
            led.setColor(LEDSubSystem.LEDState.RED);
        }
        else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
            led.setColor(LEDSubSystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubSystem.LEDState.GREEN);
        }

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());
        telemetry.update();
        super.run();
    }
}
