package org.firstinspires.ftc.teamcode.opmodes;

import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.SHOOTER_ANGLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoPoseSaver;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.*;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
@Config
@TeleOp(name = "\uD83D\uDD34 Teleop Field Centric", group = "!")
public class RedTeleOp extends CommandOpMode {
    //Constants
    private ElapsedTime snapshotTimer;
    public enum Alliance {
        RED, BLUE
    }
    public enum IntakeState {
        INTAKESTILL_ROLLERSIN, INTAKEOUT_ROLLERSOUT, INTAKEIN_ROLLERSIN, INTAKEOUT_ROLLERSIN, INTAKESTILL_ROLLERSSTILL
    }
    public enum DriveMode{
        MANUAL_CONTROL, ZERO_DEGREES, AUTO_AIM
    }

    int closeShooterTarget;
    int farShooterTarget;
    boolean isAdjustingFar = false;
    boolean isHoldingPoint = false;
    int snapshots = 0;
    double headingError;
    double headingOffset;
    int spindexerAutomoveCount = 0;
    ElapsedTime spindexerAutomoveTimeSinceLastMove = new ElapsedTime();
    boolean firstLoop = true;

    //Bulk read
    List<LynxModule> allHubs;

    private Pose holdPose = new Pose(); // Tracks where we want to stay
    final Pose GOAL_RED = new Pose(144,144);
    final Pose GOAL_BLUE = new Pose(0,144);

    final RobotConstants.BallColors[] PPG = {RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN};
    final RobotConstants.BallColors[] GPP = {RobotConstants.BallColors.GREEN, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE};
    final RobotConstants.BallColors[] PGP = {RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN, RobotConstants.BallColors.PURPLE};
    final RobotConstants.BallColors[] XXX = {RobotConstants.BallColors.UNKNOWN, RobotConstants.BallColors.UNKNOWN, RobotConstants.BallColors.UNKNOWN};

    // 2. Create a master list of all cycleable options
    final RobotConstants.BallColors[][] allMotifs = {PPG, GPP, PGP};
    int index = 0;

    //State variables
    Alliance alliance = Alliance.RED;
    RobotConstants.BallColors[] selectedMotif = new RobotConstants.BallColors[]{RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.PURPLE, RobotConstants.BallColors.GREEN};
    IntakeState intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
    DriveMode driveMode = DriveMode.MANUAL_CONTROL;
    boolean slowMode = false;
    private double gateAdjustment;
    PanelsField panelsField = PanelsField.INSTANCE;
    String image = alliance == Alliance.RED ? panelsField.getRED() : panelsField.getBLUE();

    //subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private LEDSubsystem led;
    private GateSubsystem gate;
    private ClimbSubsystem climb;
    private LimelightSubsystem limelight;
    public VoltageSensor voltageSensor;
    public GamepadEx driver1;
    public GamepadEx driver2;

    //Pedro and PID
    private Follower follower;
    public static Pose startingPose;
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;
    //Auto aligner
    public static double alignerHeadingkP = 0.5;
    public static double alignerHeadingkD = 0.06;
    public static double alignerHeadingkF = -0.05;
    PIDFController alignerHeadingPID = new PIDFController(alignerHeadingkP, 0, alignerHeadingkD, alignerHeadingkF);
    double headingPIDOutput = 0;
    double lastSeenX;
    double headingVector;
    int spOffset = 0;

    //Voltage compensation
    double currentVoltage = 14;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();

    //Timer
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime totalTimer = new ElapsedTime();

    @Override
    public void initialize () {
        initializeSystems();
        snapshotTimer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        panelsField.getField().setOffsets(panelsField.getPresets().getPEDRO_PATHING());
        panelsField.getField().setStyle("transparent", "black", 1.0);
        createBinds();
        closeShooterTarget = 505; //450;
        farShooterTarget = 620; //540;
        gateAdjustment = 0.0;
        snapshotTimer.reset();
    }

    @Override
    public void initialize_loop() {
        if (AutoPoseSaver.lastPose != null) follower.setPose(AutoPoseSaver.lastPose);
        super.initialize_loop();
    }
    void onStart() {
        if (AutoPoseSaver.lastPose != null) follower.setPose(AutoPoseSaver.lastPose);
        follower.update();
    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        if (firstLoop) {
            onStart();
            firstLoop = false;
        }
        handleTeleopDrive();
        handleLED();
        handleVoltageCompensation();
        handleBallsArrayUpdate();
        handlePanelsDrawing();

        //Update color sensors
        colorSensors.updateSensor1();
        colorSensors.updateSensor2();

        handleTelemetry();

        follower.update();
        loopTimer.reset();
        telemetry.update();
        alignerHeadingPID.setPIDF(alignerHeadingkP, 0, alignerHeadingkD, alignerHeadingkF);

        if (intakeState == IntakeState.INTAKEOUT_ROLLERSOUT || intakeState == IntakeState.INTAKEOUT_ROLLERSIN) {
            gamepad1.rumbleBlips(1);
        }

        super.run();

        if (snapshotTimer.seconds() > 5) {
            snapshots++;
            limelight.takeSnapshot();
            snapshotTimer.reset();
        }

        //LEAVE THIS AT THE END

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
    void initializeSystems() {
        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        startingPose = AutoPoseSaver.lastPose;
        if (startingPose == null) {
            startingPose = new Pose(0,0,alliance==Alliance.RED ? Math.toRadians(0) : Math.toRadians(180));
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.setPose(startingPose);
        double targetZero = alliance == Alliance.RED ? Math.toRadians(90) : Math.toRadians(-90);
        headingOffset = follower.getHeading() - targetZero;
        follower.setMaxPower(1.0);
        follower.update();
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorSensors = new ColorSensorsSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.APRILTAG);
        climb = new ClimbSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        if (Math.abs(spindexer.getWrappedPosition() - 115) < 60) {
            spindexer.set(115);
        }
        else if (Math.abs(spindexer.getWrappedPosition() - 235) < 60){
            spindexer.set(235);
        }
        else if (Math.abs(spindexer.getWrappedPosition() - 355) < 60) {
            spindexer.set(355);
        }
        else {
            spindexer.set(115);
        }
        gate.down();

        super.reset();
        lastVoltageCheck.reset();
        register(intake, shooter, spindexer, gate, colorSensors, led, limelight);
        follower.startTeleopDrive();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
    }
    void createBinds() {
        //Driver 1
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.INTAKEIN_ROLLERSIN) intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
                    else {
                        intakeState = IntakeState.INTAKEIN_ROLLERSIN;
                    }
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.INTAKEOUT_ROLLERSIN || intakeState == IntakeState.INTAKEOUT_ROLLERSOUT) {
                        intakeState = IntakeState.INTAKESTILL_ROLLERSIN;
                    }
                    else {
                        intakeState = IntakeState.INTAKEOUT_ROLLERSIN;
                    }
                    new SelectCommand(this::getIntakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new ParallelCommandGroup(
                        new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 3, true, false),
                        new InstantCommand(() -> spindexerAutomoveCount = 0)
                )
        );
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ParallelCommandGroup(
                        new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                        new InstantCommand(() -> spindexerAutomoveCount += 1)
                )
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new ParallelCommandGroup(
                    new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, -1, true, false),
                    new InstantCommand(() -> {
                        intakeState = IntakeState.INTAKEOUT_ROLLERSOUT;
                        new SelectCommand(this::getIntakeCommand).schedule();
                    }),
                    new InstantCommand(() -> spindexerAutomoveCount = 0)
                )
        );
        new Trigger( //Auto aim
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> {
                            driveMode = DriveMode.AUTO_AIM;
                        })
                );
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //slowmode
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));

        driver1.getGamepadButton(LEFT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> isHoldingPoint = true),
                        new InstantCommand(() -> follower.holdPoint(follower.getPose()))
                )
        );
        driver1.getGamepadButton(LEFT_BUMPER).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> isHoldingPoint = false),
                        new InstantCommand(() -> follower.startTeleOpDrive()),
                        new InstantCommand(() -> follower.breakFollowing())
                )
        );
        new Trigger(
                () ->
                        gamepad1.touchpad_finger_1
        ).whenActive(
                new InstantCommand(() -> {
                    driveMode = DriveMode.MANUAL_CONTROL;
                    gamepad1.rumbleBlips(2);
                    gamepad2.rumbleBlips(2);
                })
        );




        //Driver 2
//        driver2.getGamepadButton(DPAD_UP).whenPressed(
//                new InstantCommand(() -> {
//                    selectedMotif = XXX;
//                })
//        );
//        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new InstantCommand(() -> {
//                    index++;
//                    index%=3;
//                    selectedMotif = allMotifs[index];
//                })
//        );
//        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new InstantCommand(() -> {
//                    index++;
//                    index+=3;
//                    index%=3;
//                    selectedMotif = allMotifs[index];
//                })
//        );
        driver2.getGamepadButton(DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    gateAdjustment += 0.01;
                    gate.setAdjustment(gateAdjustment);
                    gamepad2.rumbleBlips(1);
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    gateAdjustment -= 0.01;
                    gate.setAdjustment(gateAdjustment);
                    gamepad2.rumbleBlips(1);
                })
        );
        driver2.getGamepadButton(LEFT_BUMPER).whenActive(  //turn off shooter
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(0);
                    gamepad2.rumbleBlips(1);
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand(() -> {
                    climb.climbUp();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    climb.climbDown();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    double targetZero = Math.toRadians(90);
                    headingOffset = follower.getHeading() - targetZero;
                    gamepad2.rumbleBlips(1);
                    gamepad1.rumbleBlips(1);
                }));
        driver2.getGamepadButton(RIGHT_STICK_BUTTON) //toggle gate
                .toggleWhenPressed(new InstantCommand(gate::up).alongWith(new InstantCommand(() -> {gamepad1.rumbleBlips(5);gamepad2.rumbleBlips(5);})), new InstantCommand(gate::down));
        driver2.getGamepadButton(RIGHT_BUMPER).whenActive(  //shooter close
                new InstantCommand(() -> {
                    shooter.setTargetLinearSpeed(closeShooterTarget);
                })
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //shooter far
                .whileActiveContinuous(new InstantCommand(() -> {
                            shooter.setTargetLinearSpeed(farShooterTarget);
                        })
                );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //shooter far
                .whileActiveContinuous(new InstantCommand(() -> {
                            shooter.setTargetLinearSpeed(farShooterTarget);
                        })
                );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //intake
                .whenActive(new InstantCommand(() -> {
                            isAdjustingFar = true;
                        })
                )
                .whenInactive(new InstantCommand(() -> {
                            isAdjustingFar = false;
                        })
                );
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (isAdjustingFar) {
                        farShooterTarget += 10;
                        gamepad2.rumbleBlips(1);
                    } else{
                        closeShooterTarget += 10;
                        gamepad2.rumbleBlips(1);
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (isAdjustingFar) {
                        farShooterTarget -= 10;
                        gamepad2.rumbleBlips(1);
                    } else{
                        closeShooterTarget -= 10;
                        gamepad2.rumbleBlips(1);
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    Pose resetPose = alliance == Alliance.RED ?
                            new Pose(7, 7, Math.toRadians(0)) :
                            new Pose(144-7, 7, Math.toRadians(180));
                    follower.setPose(resetPose);
                    gamepad2.rumbleBlips(1);
                })
        );
        new Trigger(
                () ->
                        gamepad2.touchpad_finger_2 && gamepad2.touchpad_finger_2_x < 0
        ).whenActive(
                new InstantCommand(() -> {
                        driveMode = DriveMode.ZERO_DEGREES;
                        gamepad1.rumbleBlips(2);
                        gamepad2.rumbleBlips(2);
                })
        );

        //Auto spindexer
        new Trigger(
                () ->
                        intakeState == IntakeState.INTAKEIN_ROLLERSIN &&
                        colorSensors.doesLastResultHaveBall() &&
                        (Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60) &&
                        spindexerAutomoveCount < 2 &&
                        spindexerAutomoveTimeSinceLastMove.seconds() > 0.5
        ).whenActive(
                new ParallelCommandGroup(
                        new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, false, false)
                                .withTimeout(200),
                        new InstantCommand(() -> {
                            spindexerAutomoveTimeSinceLastMove.reset();
                            spindexerAutomoveCount++;
                            if (spindexerAutomoveCount == 2) gamepad1.rumbleBlips(1);
                        }
                        )));

    }
    void handleTeleopDrive() {
        double y = driver1.getLeftY();
        double x = driver1.getLeftX();
        double rx = -driver1.getRightX() * (slowMode ? 0.3 : 1.2);

        double controlHeading = follower.getHeading() - headingOffset;

        double cos = Math.cos(-controlHeading);
        double sin = Math.sin(-controlHeading);

        double x_rotated = x * cos - y * sin;
        double y_rotated = x * sin + y * cos;

        switch(driveMode) {
            case MANUAL_CONTROL: {
                break;
            }
            case ZERO_DEGREES: {
                headingError = follower.getHeading() - Math.toRadians(0);
                headingPIDOutput = alignerHeadingPID.calculate(headingError, 0);
                rx += MathUtils.clamp(headingPIDOutput, -1, 1);
                break;
            }
            case AUTO_AIM: {
                Vector targetVector = calculateTargetVector2(follower, follower.getPose(), alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE, shooter);
                double targetHeading = targetVector.getTheta();
                shooter.setTargetLinearSpeed(targetVector.getMagnitude());

                headingError = follower.getHeading() - targetHeading;
                headingPIDOutput = alignerHeadingPID.calculate(headingError, 0);

                //MANUAL FF- NORMAL DOES NOT WORK BC SP = 0
                if (Math.abs(headingError) > Math.toRadians(0.8)) { //deadzone
                    // Apply kF in the direction of the PID output (to help it push)
                    double feedforward = Math.signum(headingError) * alignerHeadingkF;
                    headingPIDOutput += feedforward;
                }

                rx += MathUtils.clamp(headingPIDOutput, -1, 1);
                break;
            }
        }
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
        if (!isHoldingPoint) follower.setTeleOpDrive(x_rotated / denominator, y_rotated / denominator, rx / denominator, true);
    }
    void handleLED() {
        //LED Code
        if (intakeState == IntakeState.INTAKESTILL_ROLLERSIN) {
            led.setColor(LEDSubsystem.LEDState.WHITE);
        }
        else if (intakeState == IntakeState.INTAKEOUT_ROLLERSIN) {
            led.setColor(LEDSubsystem.LEDState.YELLOW);
        }
        else if (shooter.getVelocityTicks() > 300) { //shooting mode
            if (shooter.getVelocityTicks() - shooter.getTargetTicks() < -30) {
                led.setColor(LEDSubsystem.LEDState.RED);
            }
            else if (shooter.getVelocityTicks() - shooter.getTargetTicks() > 50) {
                led.setColor(LEDSubsystem.LEDState.BLUE);
            }
            else {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
        }
        else {
            double t = totalTimer.seconds();
            double min = 0.3;
            double max = 0.722;
            double amplitude = (max - min) / 2.0;
            double midpoint = (max + min) / 2.0;
            double speed = 0.6; // cycles per second — increase for faster transitions

            // Oscillate servo position smoothly with sine wave
            double position = midpoint + amplitude * Math.sin(2 * Math.PI * speed * t);
            led.setPosition(position);
        }

        //int ledBallCount = Math.toIntExact(Arrays.stream(spindexer.getBalls()).filter(ball -> !ball.equals(RobotConstants.BallColors.NONE)).count());

        if (spindexerAutomoveCount == 0) {
            led.setBlinkinLights(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        }
        else if (spindexerAutomoveCount == 1) {
            led.setBlinkinLights(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else if (spindexerAutomoveCount == 2 && colorSensors.doesLastResultHaveBall()) {
            led.setBlinkinLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (spindexerAutomoveCount == 2) {
            led.setBlinkinLights(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        }
        else {
            led.setBlinkinLights(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);
        }

    }
    void handleVoltageCompensation() {
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 30) { //check every 30ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }
    }
    void handleBallsArrayUpdate() {
        //spindexer and array logic
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 40)) {
            spindexer.handleUpdateArray(colorSensors.getIntakeSensor1Result(), colorSensors.getIntakeSensor2Result(), colorSensors.getBackResult());
        }
    }
    void handleTelemetry() {
        telemetry.addLine(alliance == Alliance.RED ? "\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34\uD83D\uDD34" : "\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35\uD83D\uDD35");
        telemetry.addData("autospindexer?", Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60);
        telemetry.addData("Loop Time", loopTimer.milliseconds());
        telemetry.addData("headingError", headingError);
        telemetry.addData("heading pid output", headingPIDOutput);
        telemetry.addData(String.format("Distance to %s goal", alliance), Math.hypot((alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getY() - follower.getPose().getY(), (alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getX() - follower.getPose().getX()));
        telemetry.addData("Mode: ", driveMode);
        telemetry.addData("Selected Motif", Arrays.toString(selectedMotif));
        telemetry.addData("Balls Array", Arrays.toString(spindexer.getBalls()));
        telemetry.addData("spindexer automove count", spindexerAutomoveCount);

        telemetry.addLine("--Spindexer--");
        telemetry.addData("PID output", spindexer.getOutput());
        telemetry.addData("PID setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("Unwrapped position", spindexer.getCurrentPosition());
        telemetry.addLine("--Shooter--");
        telemetry.addData("Target ticks", shooter.getTargetTicks());
        telemetry.addData("Actual ticks ", shooter.getVelocityTicks());
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
        telemetry.addData("Autoposesaver pose", AutoPoseSaver.lastPose);
        telemetry.addData("snapshots taken", snapshots);
        
        telemetry.addData("Spindexer Current Amps: ", spindexer.getSpindexerCurrentAmps());
        telemetry.addData("Shooter 1 Current Amps: ", shooter.getShooter1CurrentAmps());
        telemetry.addData("Shooter 2 Current Amps: ", shooter.getShooter2CurrentAmps());
        telemetry.addData("Intake Current Amps: ", intake.getIntakeCurrentAmps());

    }
    void handlePanelsDrawing() {
        FieldManager panels = panelsField.getField();
        panels.moveCursor((alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getX(), (alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getY());
        panels.setStyle("red", "black", 1.0);
        panels.circle(4.0); // Draw a 4 inch circle at the goal

        // 2. Draw the Robot (Moving position)
        Pose currentPose = follower.getPose();
        panels.moveCursor(currentPose.getX(), currentPose.getY());

        // Change color based on alignment status
        if (Math.abs(headingError) < Math.toRadians(5)) {
            panels.setStyle("green", "black", 2.0); // Green if aligned
        } else {
            panels.setStyle("yellow", "black", 2.0); // Yellow if not
        }
        panels.circle(9.0); // Draw robot (approx 18 inch width / 2)

        // 3. Draw a line from Robot to Goal (Visualizing the shot)
        // Move cursor back to robot center
        panels.moveCursor(currentPose.getX(), currentPose.getY());
        // Set line style
        panels.setStyle("transparent", "white", 1.0);
        // Draw line to goal
        panels.line((alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getX(), (alliance == Alliance.RED ? GOAL_RED : GOAL_BLUE).getY());

        // 4. Send the update
        panels.update();
    }
    /**
     * Calculate the target vector for the shooter with velocity compensation.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector(Follower follower, Pose targetPose, ShooterSubsystem shooter) {
        Pose robotPose = follower.getPose();
        Vector v_robot = follower.getVelocity(); //Assume its inches per second. in polar
        Pose robotFuturePose = robotPose.plus(new Pose(v_robot.getXComponent(), v_robot.getYComponent()).times(0.200)); //200 ms latency for shooter
        double dx = targetPose.getX() - robotFuturePose.getX();
        double dy = targetPose.getY() - robotFuturePose.getY();
        double speed = shooter.findSpeedFromDistance(Math.hypot(dx, dy));
        double horizontalSpeed = speed * Math.cos(SHOOTER_ANGLE);
        double idealHeading = Math.atan2(dy, dx);
        Vector v_target = new Vector(speed, idealHeading); //is polar
        Vector v_ball = v_target.minus(v_robot);
        return new Vector(v_ball.getMagnitude(), v_ball.getTheta() / Math.cos(SHOOTER_ANGLE));
    }

    /**
     * Calculate the target vector for the ball with velocity compensation, as well as latency in the shooter.
     * @param follower Pedro follower object
     * @param targetPose Target coordinate in Pedro system to shoot at.
     * @param shooter shooter subsystem
     * @return A vector representing the trajectory the ball should follow. The magnitude of the vector is the linear speed the ball should have.
     */
    public Vector calculateTargetVector2(Follower follower,Pose robotPose, Pose targetPose, ShooterSubsystem shooter) {
        // --- 0. CONFIGURATION ---
        // You must estimate your shooter's launch angle relative to the floor.
        // If your hood moves, calculate this based on hood position.
        // For fixed hoods, 45-60 degrees is common.
        double launchAngle = SHOOTER_ANGLE;
        double latency = 0.5; //Determine empirically

        // --- 1. GATHER CURRENT STATE ---
        Pose currentPose = robotPose;
        Vector v_robot = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();
        Vector a_robot = follower.getAcceleration();

        //Position deadzone
        if (v_robot.getMagnitude() < 2.0) { // If moving less than 2 in/s
            v_robot = new Vector(0,0);
        }
        //Angle deadzone
        if (Math.abs(angularVel) < Math.toRadians(5)) { // If rotating less than 5 deg/s
            angularVel = 0;
        }

        // Offsets (Distance in inches from center of robot to shooter)
        double shooterOffsetX = 5.0;
        double shooterOffsetY = 0.0;

        // --- 2. PREDICT ROBOT POSE (Standard Kinematics) ---
        double futureHeading = currentPose.getHeading() + (angularVel * latency);

        // Position Prediction
        double predX = currentPose.getX() + (v_robot.getXComponent() * latency);
        double predY = currentPose.getY() + (v_robot.getYComponent() * latency);

        // --- 3. CALCULATE ROBOT VELOCITY AT MUZZLE (Standard Rigid Body) ---
        double futureVx = v_robot.getXComponent();
        double futureVy = v_robot.getYComponent();

        double cosH = Math.cos(futureHeading);
        double sinH = Math.sin(futureHeading);

        // Rotated offset
        double fieldOffsetX = (shooterOffsetX * cosH) - (shooterOffsetY * sinH);
        double fieldOffsetY = (shooterOffsetX * sinH) + (shooterOffsetY * cosH);

        // Tangential velocity
        double v_tangential_x = -angularVel * fieldOffsetY;
        double v_tangential_y =  angularVel * fieldOffsetX;

        // Total Robot Velocity Components (Cartesian)
        double finalRobotVx = futureVx + v_tangential_x;
        double finalRobotVy = futureVy + v_tangential_y;

        // Convert to Polar for Pedro Vector
        double robotVelMag = Math.hypot(finalRobotVx, finalRobotVy);
        double robotVelAngle = Math.atan2(finalRobotVy, finalRobotVx);
        Vector v_robot_total = new Vector(robotVelMag, robotVelAngle);

        // --- 4. SOLVE FOR SHOOTING VECTOR ---
        double shooterMsgX = predX + fieldOffsetX;
        double shooterMsgY = predY + fieldOffsetY;

        double dx = targetPose.getX() - shooterMsgX;
        double dy = targetPose.getY() - shooterMsgY;
        double dist = Math.hypot(dx, dy);
        double idealHeading = Math.atan2(dy, dx);

        // === THE FIX STARTS HERE ===

        // A. Get the Total Exit Speed required for this distance (from your lookup table/regression)
        double totalSpeedRequired = shooter.findSpeedFromDistance(dist);

        // B. "Flatten" this speed to the 2D floor plane
        //    We only want the horizontal component for vector math
        double horizontalSpeed = totalSpeedRequired * Math.cos(launchAngle);

        // C. Create the target vector using Horizontal Speed
        Vector v_target_horizontal = new Vector(horizontalSpeed, idealHeading);

        // D. Perform Vector Subtraction in the 2D plane
        //    (Horizontal Target) - (Horizontal Robot Motion) = (Horizontal Ball Launch Vector)
        Vector v_ball_horizontal = v_target_horizontal.minus(v_robot_total);

        // E. Convert the result back to Total Exit Speed for the flywheel
        //    Total = Horizontal / cos(theta)
        double finalHorizontalSpeed = v_ball_horizontal.getMagnitude();
        double finalTotalSpeed = finalHorizontalSpeed / Math.cos(launchAngle);
        if (dist > 110) {
            finalTotalSpeed = 620;
        }

        // Return a Vector with the NEW Total Speed and the CORRECTED heading
        return new Vector(finalTotalSpeed, v_ball_horizontal.getTheta());
    }
    /**
     * Calculates the smallest difference between two angles in radians.
     * @return a double between -PI and +PI.
     */
    public double getAngleDifference(double targetAngle, double currentAngle) {
        double difference = targetAngle - currentAngle;

        // Normalize the angle to be within -PI to +PI
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference <= -Math.PI) {
            difference += 2 * Math.PI;
        }
        return difference;
    }
    public Command getIntakeCommand() {
        switch (intakeState) {
            case INTAKEIN_ROLLERSIN:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN);
                });
            case INTAKEOUT_ROLLERSOUT:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSOUT);
                });
            case INTAKEOUT_ROLLERSIN:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSIN);
                });
            case INTAKESTILL_ROLLERSIN:
            default:
                return new InstantCommand(() -> {
                    intake.set(IntakeSubsystem.IntakeState.INTAKESTILL_ROLLERSIN);
                });
        }
    }
}