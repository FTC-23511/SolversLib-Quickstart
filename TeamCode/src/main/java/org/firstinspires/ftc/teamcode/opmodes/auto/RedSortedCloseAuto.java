package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.AutoPoseSaver;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.commands.LoadBallCommand;
import org.firstinspires.ftc.teamcode.commands.LoadMotifCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
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

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "\uD83D\uDD34 Red Close 12 Sort", group = "angryBirds", preselectTeleOp = "RedTeleOp")
public class RedSortedCloseAuto extends CommandOpMode {
    public static class Paths {
        //close autos
        public PathChain shootClosePreload;
        public PathChain intakeSecondRowClose;
        public PathChain shootSecondRowClose;
        public PathChain hitGateSecond;
        public PathChain intakeFirstRowClose;
        public PathChain shootFirstRowClose;
        public PathChain hitGateFirst;
        public PathChain intakeThirdRowClose;
        public PathChain shootThirdRowClose;

        public static class Poses {
            public static final Pose LAUNCH = new Pose(86.8, 88.2, 0.715585);
            public static final Pose START = new Pose(129,115,Math.toRadians(180));
            public static final Pose GATE = new Pose(132, 66);
            public static final Pose PARK_LAUNCH = new Pose(87.79745,110.10889, Math.toRadians(20));
        }

        public Paths(Follower follower) {
            shootClosePreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.START, Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Poses.START.getHeading(), Math.toRadians(43))//on purpose
                    .build();
            intakeSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(87.6, 43),
                                    new Pose(126.13, 52)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            hitGateSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.13, 52), Poses.GATE)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            shootSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.GATE,
                                    new Pose(91.5, 56),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Poses.LAUNCH.getHeading())
                    .build();

            intakeFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(100, 79.5),
                                    new Pose(122, 84)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            hitGateFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116, 84), Poses.GATE)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            shootFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126, 84), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.LAUNCH.getHeading())
                    .build();

            intakeThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(83, 11),
                                    new Pose(128, 36)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            shootThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 36), Poses.PARK_LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.PARK_LAUNCH.getHeading())
                    .build();
        }
    }

    //Since intakeartifacts is called at very different times (called when on the gate, called before driving to the row of balls)
    //we might need to split it up.
    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                new WaitForColorCommand(colorsensor).withTimeout(3000),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(100),
                new WaitForColorCommand(colorsensor).withTimeout(500),
                new WaitCommand(100),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitForColorCommand(colorsensor).withTimeout(500)
        );
    }

    List<LynxModule> allHubs;
    //Selectiopn
    private enum AUTOS {
        GATE_ONCE, INTAKE_GATE
    }
    final AUTOS CURRENTAUTO = AUTOS.GATE_ONCE;

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
    public static Pose startingPose = Paths.Poses.START;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    private LimelightSubsystem limelight;

    //debugging
    int debug = 0;
    InstantCommand setCount(int n) {
        return new InstantCommand(() -> debug = n);
    }
    void scanMotif() {
        limelight.takeSnapshot("MOTIF");
        Object motifid = limelight.detectMotif(limelight.getResult());
        if (motifid != null) {
            switch ((int) motifid) {
                case 21:
                    motif = new RobotConstants.BallColors[]{GREEN, PURPLE, PURPLE}; //changed to actual motif
                    break;
                case 22:
                    motif = new RobotConstants.BallColors[]{PURPLE, GREEN, PURPLE};
                    break;
                case 23:
                    motif = new RobotConstants.BallColors[]{PURPLE, PURPLE, GREEN}; //changed to actual motif
                    break;
            }
        }
    }
    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        intake.set(IntakeSubsystem.IntakeState.INTAKESTILL_ROLLERSSTILL);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        gate.down();
        led = new LEDSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.APRILTAG);
        spindexer.setPIDCoefficients(0.0155, 0, 0.00055, 0);
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        colorsensor.updateBack();
        lastVoltageCheck.reset();
        Paths paths = new Paths(follower);
        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(115);
        SequentialCommandGroup nine_sorted = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetTicks(1140);
                    gate.down();
                    spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});
                }),
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootClosePreload, true)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6).andThen(new InstantCommand(this::scanMotif)),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.8).andThen(new InstantCommand(this::scanMotif))
                ),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
                new WaitCommand(200),
                new DeferredCommand(() -> new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false)),

                //Second row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeSecondRowClose)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, GREEN, PURPLE});}),
                new FollowPathCommand(follower, paths.hitGateSecond).withTimeout(1500),
                new WaitCommand(1000),
                new FollowPathCommand(follower, paths.shootSecondRowClose, true)
                        .alongWith(new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(gate::up),
                                new WaitCommand(200),
                                new DeferredCommand(() -> new LoadMotifCommand(spindexer, motif)),
                                new InstantCommand(gate::down)
                        )),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif)),

                //First row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeFirstRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});}),
                //first row
                new FollowPathCommand(follower, paths.shootFirstRowClose, true)
                        .alongWith(new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(gate::up),
                                new WaitCommand(200),
                                new DeferredCommand(() -> new LoadMotifCommand(spindexer, motif)),
                                new InstantCommand(gate::down)
                        )),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif)),

                //intake third row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        new WaitCommand(2000)
                                .andThen(intakeArtifacts())
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, PURPLE, GREEN});}),

                //shoot third row
                new InstantCommand(() -> {
                    shooter.setTargetTicks(1100);
                }),
                new FollowPathCommand(follower, paths.shootThirdRowClose, true)
                        .alongWith(new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(gate::up),
                                new WaitCommand(200),
                                new DeferredCommand(() -> new LoadMotifCommand(spindexer, motif)),
                                new InstantCommand(gate::down)
                        )),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif))
        );

        schedule(new RunCommand(() -> follower.update()));
        schedule(new SequentialCommandGroup(nine_sorted));

    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorsensor.getIntakeSensor1Result(), colorsensor.getIntakeSensor2Result(), colorsensor.getBackResult());
        }
        if (shooter.getVelocityTicks() - shooter.getTargetTicks() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getVelocityTicks() - shooter.getTargetTicks() > 50) {
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

        telemetry.addData("Debug Counter", debug);
        telemetry.addData("Detected Motif", Arrays.toString(motif));

//        telemetry.addData("spindexer output", spindexer.getOutput());
//        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
//        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("spindexer's balls", Arrays.toString(spindexer.getBalls()));

        telemetry.addData("------------------",0);

        telemetry.addData("shooter target velocity", shooter.getTargetTicks());
        telemetry.addData("shooter actual velocity", shooter.getVelocityTicks());

        telemetry.addData("------------------",0);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",0);
        currentPose = follower.getPose().plus(
                new Pose(-2,0) //DO NOT MIRROR THIS! INVERT THE X AXIS *ONLY*
        ); //Auto->teleop offset
        AutoPoseSaver.lastPose = currentPose;
        timer.reset();
        telemetry.update();
        super.run();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

    }

    @Override
    public void end() {
        AutoPoseSaver.lastPose = currentPose;
        super.end();
    }
}
