package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.UNKNOWN;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
import org.firstinspires.ftc.teamcode.opmodes.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;
@Autonomous
public class Red18Auto extends CommandOpMode {
    //Generated from december_18.pp file.
    //Note: I consolidaated shootfirstrow1/2 into one pathchain for optimization.
    //also done to shootthirdrow
    public static class Paths {

        public PathChain shootPreload;
        public PathChain intakeSecondRow;
        public PathChain shootSecondRow;
        public PathChain intakeRamp;
        public PathChain shootRamp;
        public PathChain intakeRamp2;
        public PathChain shootRamp2;
        public PathChain intakeFirstRow;
        public PathChain shootFirstRow;
        public PathChain intakeThirdRow;
        public PathChain shootThirdRow;

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
                new WaitForColorCommand(colorSensors).withTimeout(1500),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new WaitForColorCommand(colorSensors).withTimeout(1500),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new WaitForColorCommand(colorSensors).withTimeout(1500)
        );
    }
    public Pose currentPose;

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private ElapsedTime loopTimer = new ElapsedTime();
    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(122.361,121.175,Math.toRadians(49));
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private GateSubsystem gate;
    private LEDSubsystem led;
    private LimelightSubsystem limelight;
    private RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{UNKNOWN,UNKNOWN,UNKNOWN};

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
        colorSensors = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightSubsystem(hardwareMap);
        lastVoltageCheck.reset();
        led = new LEDSubsystem(hardwareMap);
        Paths paths = new Paths(follower);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorSensors, led, gate);
        spindexer.set(75);
        SequentialCommandGroup autonomous = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetLinearSpeed(1200);
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
        handleLED();
        handleSpindexer();

        //Update color sensors
        colorSensors.updateSensor1();
        colorSensors.updateSensor2();
        colorSensors.updateBack(); //Update every time.... for now .......

        handleTelemetry();

        follower.update();
        telemetry.update();
        loopTimer.reset();
        super.run();


    }

    @Override
    public void end() {
        blackboard.put("endpose", currentPose);
        super.end();
    }
    void handleLED() {
        if (shooter.getActualVelocity() > 300) { //shooting mode
            if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
                led.setColor(LEDSubsystem.LEDState.RED);
            } else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
                led.setColor(LEDSubsystem.LEDState.BLUE);
            } else {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
        }
    }
    void handleSpindexer() {
        //spindexer and array logic
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorSensors.getIntakeSensor1Result(), colorSensors.getIntakeSensor2Result(), colorSensors.getBackResult());
        }
    }
    void handleTelemetry() {
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("Balls Array", Arrays.toString(spindexer.getBalls()));
        telemetry.addLine("--Spindexer--");
        telemetry.addData("PID output", spindexer.getOutput());
        telemetry.addData("PID setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("Unwrapped position", spindexer.getCurrentPosition());
        telemetry.addLine("--Shooter--");
        telemetry.addData("Target velocity", shooter.getTargetVelocity());
        telemetry.addData("Actual velocity ", shooter.getActualVelocity());
        telemetry.addData("Linear speed ", shooter.getFlywheelLinearSpeed());
        telemetry.addLine("--Color Sensors--");
        NormalizedRGBA val1 = colorSensors.getIntakeSensor1Result();
        NormalizedRGBA val2 = colorSensors.getIntakeSensor2Result();
        NormalizedRGBA valBack = colorSensors.getBackResult();
        // -- Sensor 1 (Intake) --
        String color1 = "None";
        float[] hsv1 = {0,0,0};
        if (val1 != null) {
            hsv1 = ColorSensorsSubsystem.rgbToHsv(val1);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val1)) color1 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val1)) color1 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val1)) color1 = "White";
        }
        telemetry.addData("Intake 1", "[%s] H:%.0f S:%.2f V:%.2f", color1, hsv1[0], hsv1[1], hsv1[2]);
        // -- Sensor 2 (Intake) --
        String color2 = "None";
        float[] hsv2 = {0,0,0};
        if (val2 != null) {
            hsv2 = ColorSensorsSubsystem.rgbToHsv(val2);
            if (ColorSensorsSubsystem.colorIsPurpleIntake(val2)) color2 = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenIntake(val2)) color2 = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(val2)) color2 = "White";
        }
        telemetry.addData("Intake 2", "[%s] H:%.0f S:%.2f V:%.2f", color2, hsv2[0], hsv2[1], hsv2[2]);
        // -- Back Sensor (Uses Back-specific logic) --
        String colorBack = "None";
        float[] hsvBack = {0,0,0};
        if (valBack != null) {
            hsvBack = ColorSensorsSubsystem.rgbToHsv(valBack);
            if (ColorSensorsSubsystem.colorIsPurpleBack(valBack)) colorBack = "Purple";
            else if (ColorSensorsSubsystem.colorIsGreenBack(valBack)) colorBack = "Green";
            else if (ColorSensorsSubsystem.colorIsWhite(valBack)) colorBack = "White";
        }
        telemetry.addData("Back", "[%s] H:%.0f S:%.2f V:%.2f", colorBack, hsvBack[0], hsvBack[1], hsvBack[2]);
        telemetry.addLine("--Pedro--");
        telemetry.addData("Position ", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("Heading ", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("Slow mode", slowMode);

    }
}
