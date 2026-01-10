package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.UNKNOWN;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSortedBallsCommandSequence;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class Red18SortOverflowAuto extends CommandOpMode {
    //Generated from december_18.pp file.
    //Note: I consolidaated shootfirstrow1/2 into one pathchain for optimization.
    //also done to shootthirdrow
    public static class Paths {

        public PathChain shootPreload;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain intakeFirstRow;
        public PathChain shootFirstRow;
        public PathChain intakeRamp;
        public PathChain shootRamp;

        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 136.000), new Pose(88.400, 81.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .addPath(
                            new BezierLine(new Pose(88.400, 81.800), new Pose(88.400, 81.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(49))
                    .build();
            intakeSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.400, 81.800),
                                    new Pose(78.000, 55.000),
                                    new Pose(136.000, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(0))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136.000, 59.000),
                                    new Pose(78.000, 55.000),
                                    new Pose(88.400, 81.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(49))
                    .build();

            intakeThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.400, 81.800),
                                    new Pose(90.300, 33.100),
                                    new Pose(127.700, 35.600)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.700, 35.600), new Pose(88.400, 81.800))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierLine(new Pose(88.400, 81.800), new Pose(125.000, 83.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()

                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.000), new Pose(88.400, 81.800))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierLine(new Pose(88.400, 81.800), new Pose(88.400, 81.800))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(49))
                    .build();
            intakeRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.400, 81.800),
                                    new Pose(89.000, 58.000),
                                    new Pose(133.500, 51.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(50))
                    .build();

            shootRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.500, 51.000),
                                    new Pose(89.000, 58.000),
                                    new Pose(88.400, 81.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(49))
                    .build();
            
        }
    }

    //Since intakeartifacts is called at very different times (called when on the gate, called before driving to the row of balls)
    //we might need to split it up.
    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING)),
                new WaitForColorCommand(colorsensor).withTimeout(1500),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new WaitForColorCommand(colorsensor).withTimeout(1500),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new WaitForColorCommand(colorsensor).withTimeout(1500)
        );
    }
    public Pose currentPose;
    public RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{PURPLE, PURPLE,PURPLE};

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(122.361,121.175,Math.toRadians(49));
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    private LimelightSubsystem limelight;

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.APRILTAG);
        lastVoltageCheck.reset();
        led = new LEDSubsystem(hardwareMap);
        Paths paths = new Paths(follower);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(75);
        SequentialCommandGroup autonomous = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetLinearSpeed(1200);
                    gate.down();
                    follower.setMaxPower(0.8);
                }),
                //Preload
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.shootPreload, true),
                        new InstantCommand(() -> {
                            if (follower.getPathCompletion() > 0.4) {
                                Object motifid = limelight.detectMotif(limelight.getResult());
                                if (motifid != null) {
                                    switch ((int) motifid) {
                                        case 21:
                                            motif = new RobotConstants.BallColors[]{GREEN, PURPLE, PURPLE};
                                            break;
                                        case 22:
                                            motif = new RobotConstants.BallColors[]{PURPLE, GREEN, PURPLE};
                                            break;
                                        case 23:
                                            motif = new RobotConstants.BallColors[]{PURPLE, PURPLE, GREEN};
                                            break;
                                    }
                                }
                            }
                        })
                ),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif),

                //Second row
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeSecondRow)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING))),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootSecondRow, true),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif),

                //First row
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeFirstRow).withTimeout(3000),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootFirstRow),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif),

                //Third row
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRow).withTimeout(3000),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootThirdRow),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif),

                //Ramp cycle
                new FollowPathCommand(follower, paths.intakeRamp, true),
                intakeArtifacts().withTimeout(3000),
                new FollowPathCommand(follower, paths.shootRamp, true),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif),

                //Ramp cycle
                new FollowPathCommand(follower, paths.intakeRamp, true),
                intakeArtifacts().withTimeout(3000),
                new FollowPathCommand(follower, paths.shootRamp, true),
                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, motif)

                //later: park?
        );
        schedule(
                new RunCommand(() -> follower.update()),
                autonomous
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
