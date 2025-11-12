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
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.WaitForRobotStuckCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous(name = "Blue Auto", group = "angryBirds", preselectTeleOp = "Alpha Teleop")
public class BlueAuto extends CommandOpMode {
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

    //subsytems and pedro

    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(123.36079077429983,122.17462932454696,Math.toRadians(45)).mirror(); //find actual statring pos
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSubsystem colorsensor;
    private LEDSubsystem led;

    public void buildPaths(Follower follower) {
        follower.setStartingPose(startingPose);
        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.361, 122.175).mirror(), new Pose(84, 84).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84, 84).mirror(), new Pose(101.000, 84.000).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(101.000, 84.000).mirror(), new Pose(130.000, 84.000).mirror())
                )
                .setTangentHeadingInterpolation()
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 84.000).mirror(), new Pose(84, 84).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84,  84).mirror(), new Pose(95.000, 60.000).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95.000, 60.000).mirror(), new Pose(140.000, 60.000).mirror())
                )
                .setTangentHeadingInterpolation()
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(140.000, 60.000).mirror(), new Pose(125.000, 60.000).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 60.000).mirror(), new Pose(84, 84).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build()
        );

        paths.add(follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84, 84).mirror(), new Pose(84.000, 108.000).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build()
        );

    }
    //preset command methods
    public SequentialCommandGroup shootArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new ParallelRaceGroup(
                        new WaitForShooterCommand(shooter),
                        new WaitCommand(1000)
                ),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new ParallelRaceGroup(
                        new WaitForShooterCommand(shooter),
                        new WaitCommand(1000)
                ),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new ParallelRaceGroup(
                        new WaitForShooterCommand(shooter),
                        new WaitCommand(1000)
                )
        );
    }

    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(1500)
                ),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                ),
                new InstantCommand(() -> spindexer.advanceSpindexer()),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                ),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.REVERSE))
        );
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led);

        //init paths
        buildPaths(follower);


        //schedule commands
        //one cycle = 3 balls + shoot given starting position is right at the shooting spot
        //pp file is editable but you have to update the buildPath

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new InstantCommand(() -> {shooter.setTargetVelocity(1200);}), //start shoot
                        new FollowPathCommand(follower, paths.get(0), true), //drive to shooting pos
                        new InstantCommand(() -> follower.setMaxPower(1.0)),
                        new WaitCommand(500),
                        shootArtifacts(),

                        //cycle one
                        new FollowPathCommand(follower, paths.get(1), true), //drives to balls and lines itself up to intake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(0.3)),
                                intakeArtifacts(),
                                new ParallelRaceGroup(
                                        new FollowPathCommand(follower, paths.get(2), true), //drive and pick up balls
//                                        new WaitForRobotStuckCommand(follower) //does not work for some reason
                                        new WaitCommand(5000)
                                )

                        ),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, paths.get(3), true), // returning to shooting pos
                        //needs time for shooter to ramp up
                        shootArtifacts(),

                        //cycle two
                        new FollowPathCommand(follower, paths.get(4), true), //drives to balls and lines itself up to intake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> follower.setMaxPower(0.4)),
                                intakeArtifacts(),
                                new ParallelRaceGroup(
                                        new FollowPathCommand(follower, paths.get(5), true), //drive and pick up balls
//                                        new WaitForRobotStuckCommand(follower)
                                        new WaitCommand(5000)
                                )
                        ),
                        //needs extra step to back out from the wall because it will collide with the exit of the ramp
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, paths.get(6), true),

                        new FollowPathCommand(follower, paths.get(7), true), //return to shooting pos
                        shootArtifacts(),

                        //move off shooting line so that you get extra points theoretically
                        new FollowPathCommand(follower, paths.get(8), true),

                        new InstantCommand(() -> {shooter.setTargetVelocity(0);})
                )
        );


    }
    @Override
    public void run() {
        if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
            led.setColor(LEDSubsystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubsystem.LEDState.GREEN);
        }

        telemetry.addData("stuck?", follower.isRobotStuck());

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading (deg): %.4f", Math.toDegrees(follower.getPose().getHeading())));

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());
        telemetry.addData("green color detected?", Arrays.toString(colorsensor.senseColor(1)));
        telemetry.addData("green color detected?", colorsensor.checkIfGreen(1));
        telemetry.addData("purple color detected?", colorsensor.checkIfPurple(1));

        follower.update();
        telemetry.update();
        super.run();
    }
}
