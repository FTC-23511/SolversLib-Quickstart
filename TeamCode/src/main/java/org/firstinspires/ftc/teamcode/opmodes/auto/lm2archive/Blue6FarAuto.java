package org.firstinspires.ftc.teamcode.opmodes.auto.lm2archive;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@Disabled
@Config
@Autonomous(name = "Blue 6ball preload + hp🦅, 5s delay", group = "angryBirds", preselectTeleOp = "Teleop")
public class Blue6FarAuto extends CommandOpMode {
    //paths
    public static class Paths {
        public PathChain to69Deg;
        public PathChain toHpZone;
        public PathChain grabHpBalls;
        public PathChain returnToFarZone;
        public PathChain leaveZone;

        public Paths(Follower follower) {
            to69Deg = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-87.000, 8.294), new Pose(144-86.000, 16.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180-69))
                    .build();
            toHpZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-86.000, 18.000), new Pose(144-129.882, 9.882))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            grabHpBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(144-129.882, 9.882),
                                    new Pose(144-138.1764705882353, 19.764705882352935)
                            )
                    )
                    .addPath(
                            new BezierLine(
                                    new Pose(144-138.1764705882353, 19.764705882352935),
                                    new Pose(144-134.29411764705884, 5.470588235294114)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180+10))
                    .build();

            returnToFarZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-134.29411764705884, 5.470588235294114), new Pose(144-87.000, 8.200))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180-69))
                    .build();

            leaveZone = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-87.000, 8.200), new Pose(144-110.000, 10.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180-69))
                    .build();
        }
    }
    Paths paths;
    public Pose currentPose;

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(144-87,8.294117647058826,Math.toRadians(180-90));
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    public void buildPaths(Follower follower) {
        follower.setStartingPose(startingPose);
        paths = new Paths(follower);
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
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        lastVoltageCheck.reset();
        led = new LEDSubsystem(hardwareMap);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(75);
        shooter.setHood(0.45);
        gate.down();

        //init paths
        buildPaths(follower);


        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> { //immediately set shooter to max speed.
                            shooter.setTargetVelocity(1500);
                        }),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, paths.to69Deg, 0.3),
                                new WaitCommand(5000)
                        ),
                        new InstantCommand(() -> { //launch all 3 balls
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                        }),
                        new WaitCommand(1500),
                        new InstantCommand(() -> {
                            intake.set(IntakeSubsystem.IntakeState.INTAKING);
                            shooter.setTargetVelocity(0); //Since shooter might launch into hp turn off shooter to be nice :)
                        }),
                        new ParallelRaceGroup( //Do both, end when a or b finishes first:
                                new ParallelCommandGroup( //a. both paths finish following with the timeout
                                        new FollowPathCommand(follower, paths.toHpZone, 0.7)
                                                .withTimeout(1400),
                                        new FollowPathCommand(follower, paths.grabHpBalls, 0.5)
                                                .withTimeout(5000)
                                ),
                                new SequentialCommandGroup( //b. the ball intaking sequence finishes.
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor)
                                )
                        ),
                        new FollowPathCommand(follower, paths.returnToFarZone)
                                .withTimeout(5000)
                                .alongWith(new InstantCommand(() -> shooter.setTargetVelocity(1500))) //we should use these decorators more
//                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.REVERSE))) //reverse so we dont get 4?
                        ,
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                        }),
                        new WaitCommand(1500),
                        new FollowPathCommand(follower, paths.leaveZone)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING))) //maybe we can randomly get another?

                )
        );


    }
    @SuppressLint("DefaultLocale")
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
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }


        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("is spindexer ready to read color ", spindexer.availableToSenseColor());
        telemetry.addData("spindexer's balls", spindexer.getBalls());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());

        telemetry.addData("------------------",null);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",null);
        currentPose = follower.getPose();
        timer.reset();
        telemetry.update();
        super.run();

    }

    @Override
    public void end() {
        blackboard.put("endpose", currentPose);
        super.end();
    }
}
