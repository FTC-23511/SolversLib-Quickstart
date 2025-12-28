package org.firstinspires.ftc.teamcode.opmodes.auto;

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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class Red18Auto extends CommandOpMode {
    //Generated from december_18.pp file.
    //Note: I consolidaated shootfirstrow1/2 into one pathchain for optimization.
    //also done to shootthirdrow
    public static class Paths {

        public PathChain shootPreload;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain intakeRamp;
        public double wait1;
        public PathChain shootRamp;
        public PathChain intakeRamp2;
        public double wait2;
        public PathChain shootRamp2;
        public PathChain intakeFirstRow;
        public PathChain shootFirstRow;
        public PathChain shootFirstRow2;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;
        public PathChain shootThirdRow2;

        public Paths(Follower follower) {
            shootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.361, 121.175), new Pose(88.400, 81.800))
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

            intakeRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.400, 81.800),
                                    new Pose(89.000, 58.000),
                                    new Pose(133.500, 61.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(50))
                    .build();

            wait1 = 3000;

            shootRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.500, 61.000),
                                    new Pose(89.000, 58.000),
                                    new Pose(88.400, 81.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(49))
                    .build();

            intakeRamp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.400, 81.800),
                                    new Pose(89.000, 58.000),
                                    new Pose(133.500, 61.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(50))
                    .build();

            wait2 = 3000;

            shootRamp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.500, 61.000), new Pose(88.400, 81.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(49))
                    .build();

            intakeFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.400, 81.800), new Pose(125.000, 83.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.000, 83.000), new Pose(88.400, 81.800))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierLine(new Pose(88.400, 81.800), new Pose(88.400, 81.8001)) //0.0001 offset to avoid div by 0
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(49))
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
                            new BezierLine(new Pose(127.700, 35.600), new Pose(90.000, 110.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierLine(new Pose(90.000, 110.000), new Pose(90.000, 110.0001)) //0.0001 offset to avoid div by 0
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(35))
                    .build();
        }
    }
    //This command group is supposed to end once all 3 balls are released. If the WaitUntilCommand does not function right, then replace
    //it with an empircally tuned WaitCommand.
    public SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
                new MoveSpindexerCommand(spindexer, gate, 3, true),
                new WaitCommand(100), //wait for spindexer to start moving
                new WaitUntilCommand(() -> spindexer.isLowVelocity() && spindexer.isNearTargetPosition())
        );
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
    private RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{UNKNOWN,UNKNOWN,UNKNOWN};

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
        //todo add limelight to all the autos once branch is merged
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
                    shooter.setHood(0.45); //Placeholder
                    gate.down();
                    follower.setMaxPower(0.8);
                }),
                //Preload
                new FollowPathCommand(follower, paths.shootPreload, true),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
                shoot(),

                //Second row
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeSecondRow)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING))),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootSecondRow, true),
                shoot(),

                //Ramp cycle
                new FollowPathCommand(follower, paths.intakeRamp, true),
                intakeArtifacts().withTimeout(3000),
                new FollowPathCommand(follower, paths.shootRamp, true),
                shoot(),

                //Ramp cycle
                new FollowPathCommand(follower, paths.intakeRamp, true),
                intakeArtifacts().withTimeout(3000),
                new FollowPathCommand(follower, paths.shootRamp, true),
                shoot(),

                //First row
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeFirstRow).withTimeout(3000),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootFirstRow),
                shoot(),

                //Third row + park
                new ParallelRaceGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRow).withTimeout(3000),
                        intakeArtifacts()
                ),
                new FollowPathCommand(follower, paths.shootThirdRow),
                shoot()
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
