package org.firstinspires.ftc.teamcode.opmodes;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.RobotConstants.Motifs.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants.*;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.ScanAndUpdateBallsCommand;
import org.firstinspires.ftc.teamcode.commands.ScheduleGateCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

@TeleOp (name = "Alpha Teleop", group = "!")
public class AlphaTeleOp extends CommandOpMode {
    public Motifs motifs = PPG;

    //pedro
    private Follower follower;
    public static Pose startingPose = new Pose(0,0,0);
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;

    //subsystems
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorSensors;
    private LEDSubsystem led;
    private GateSubsystem gate;
    private CameraSubsystem camera;

    //gamepads
    public GamepadEx driver1;
    public GamepadEx driver2;

    //vision
    boolean cameraInitialized = false;

    //autodrive
    private boolean manualControl = true;
    private void setSavedPose(Pose pose) {
        savedPose = pose;
        gamepad1.rumbleBlips(1);
    }
    private void goToSavedPose() {
        Pose currentPose = follower.getPose();
        manualControl = false;
        follower.holdPoint(savedPose);
        gamepad1.rumbleBlips(3);
    }

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();

    //variable shooter target
    double closeShooterTarget = 1200;
    double farShooterTarget = 1500;

    //looptime
    private ElapsedTime timer = new ElapsedTime();

    //spindexer adjustment
    private int spindexerAdjustmentCount = 0;

    //intake state machine
    public enum IntakeState {
        STOP, FORWARD, REVERSE
    }
    private IntakeState intakeState = IntakeState.STOP;
    public Command intakeCommand() {
        switch (intakeState) {
            case FORWARD:
                return new InstantCommand(() -> {
                    intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING);
                });
            case REVERSE:
                return new InstantCommand(() -> {
                    intake.setSpeed(IntakeSubsystem.IntakeState.REVERSE);
                });
            case STOP:
            default:
                return new InstantCommand(() -> {
                    intake.setSpeed(IntakeSubsystem.IntakeState.STILL);
                });
        }
    }


    //point to april tag
    public static double headingkP = 0.1;
    public static double headingkD = 0.0001;
    PIDController headingPID = new PIDController(headingkP, 0, headingkD);
    double lastSeenX;
    double headingVector;

    @Override
    public void initialize () {
        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorSensors = new ColorSensorsSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        camera = new CameraSubsystem();
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        super.reset();
        lastVoltageCheck.reset();
        register(intake, shooter, spindexer, gate, colorSensors, led, camera);

        spindexer.set(75);
        shooter.setHood(0.45);
        gate.down();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //pedro and gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
//        pathChainSupplier = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, savedPose)))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, savedPose::getHeading, 0.8))
//                .build();
        //command binding
        SelectCommand intakeSelectCommand = new SelectCommand(this::intakeCommand);

        //Driver 1
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.FORWARD) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.FORWARD;
                    new SelectCommand(this::intakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    if (intakeState == IntakeState.REVERSE) intakeState = IntakeState.STOP;
                    else intakeState = IntakeState.REVERSE;
                    new SelectCommand(this::intakeCommand).schedule();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new MoveSpindexerCommand(spindexer, gate, 1, true)
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new MoveSpindexerCommand(spindexer, gate, -1, true)
        );
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    goToSavedPose();
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    setSavedPose(follower.getPose());
                })
        );
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    gamepad1.rumbleBlips(3);
                    manualControl = false;
                })
        );
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));
        //Driver 2
        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                gate::up
        );
        driver2.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                gate::down
        );
        driver2.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand(() -> {
                    shooter.setHood(clamp(shooter.getHoodPos() + 0.01, 0.0, 1.0));
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    shooter.setHood(clamp(shooter.getHoodPos() - 0.01, 0.0, 1.0));
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> {
                    if (motifs == PPG) {
                        motifs = GPP;
                    }
                    if (motifs == GPP) {
                        motifs = PGP;
                    }
                    if (motifs == PGP) {
                        motifs = PPG;
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    if (motifs == PPG) {
                        motifs = PGP;
                    }
                    if (motifs == PGP) {
                        motifs = GPP;
                    }
                    if (motifs == GPP) {
                        motifs = PPG;
                    }
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed( //close distance
                new InstantCommand(() -> {
                    shooter.setTargetVelocity(closeShooterTarget);
                })
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //far distance
                    .whileActiveContinuous(new InstantCommand(() -> {
                        shooter.setTargetVelocity(farShooterTarget);
                    })
                );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(  //turn off shooter
                new InstantCommand(() -> {
                    shooter.setTargetVelocity(0);
                    gamepad2.rumbleBlips(1);
                })
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) //intake
                .whenActive(new InstantCommand(() -> {
                    driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                            new InstantCommand(() -> {
                                farShooterTarget += 20;
                                gamepad2.rumbleBlips(1);
                            })
                    );
                    driver2.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                            new InstantCommand(() -> {
                                farShooterTarget -= 20;
                                gamepad2.rumbleBlips(1);
                            })
                    );
                        })
                )
                .whenInactive(new InstantCommand(() -> {
                    driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                            new InstantCommand(() -> {
                                closeShooterTarget += 20;
                                gamepad2.rumbleBlips(1);
                            })
                    );
                    driver2.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                            new InstantCommand(() -> {
                                closeShooterTarget -= 20;
                                gamepad2.rumbleBlips(1);
                            })
                    );
                        })
                );
    }

    @Override
    public void run() {
        if (!cameraInitialized) {
            camera.setAprilTagProcessor(new AprilTagProcessor.Builder()
                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.
                    .build());
            camera.setVisionPortal(new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(camera.getAprilTagProcessor())
                    .build()
            );
            cameraInitialized = true;
        }
        gate.down(); //temp fix

        //While intake is on, scan color sensors
        if (!intakeState.equals(IntakeState.STOP) && spindexer.availableToSenseColor()) {
            schedule(new ScanAndUpdateBallsCommand(spindexer, colorSensors));
        }


        //Drivetrain code
        if (manualControl) {
            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = -driver1.getRightX() * (slowMode?0.3:1);
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(y / denominator, x / denominator, rx / denominator, true);
        } else {
            List<AprilTagDetection> detections = camera.detectAprilTags();
            if (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2) {
                manualControl = true;
            }
            double x = -driver1.getLeftX();
            double y = driver1.getLeftY();
            double rx = 0;
            if (camera.detectGoalXDistance(detections) != null) {
                lastSeenX = (double) camera.detectGoalXDistance(detections);
                headingVector = -headingPID.calculate(lastSeenX, 0);
                rx = headingVector; //replace 100 (placeholder) with camera april tag x
            } else {
                rx = -driver1.getRightX() * (slowMode?0.3:1);
            }
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1.0);
            follower.setTeleOpDrive(y / denominator, x / denominator, rx / denominator, true);
        }
        follower.update();

        //LED Code
        if (shooter.getActualVelocity() > 300) { //shooting mode
            if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
                led.setColor(LEDSubsystem.LEDState.RED);
            }
            else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
                led.setColor(LEDSubsystem.LEDState.BLUE);
            }
            else {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
        }
        else if (!intakeState.equals(IntakeState.STOP)){ //intaking mode
            if (colorSensors.checkIfGreen(3)) {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
            else if (colorSensors.checkIfPurple(3)) {
                led.setColor(LEDSubsystem.LEDState.VIOLET);
            }
            else if (colorSensors.checkIfWhite(3)){
                led.setColor(LEDSubsystem.LEDState.WHITE);
            }
            else {
                led.setColor(LEDSubsystem.LEDState.YELLOW);
            }
        } else {
            led.setColor(LEDSubsystem.LEDState.OFF);
        }

        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }

        telemetry.addData("BALLS", Arrays.toString(spindexer.getBalls()));

        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("current motif ", motifs);
        telemetry.addData("spindexer output ", spindexer.getOutput());
        telemetry.addData("spindexer setpoint ", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos ", spindexer.getCurrentPosition());
        telemetry.addData("spindexer tick adjustment degrees ", spindexerAdjustmentCount);
        telemetry.addData("is spindexer ready to read color ", spindexer.availableToSenseColor());

        telemetry.addData("------------------",null);

        telemetry.addData("last seen goal x pos ", lastSeenX);
        telemetry.addData("last pid power to heading", headingVector);

        telemetry.addData("------------------",null);

        telemetry.addData("shooter close amount ", closeShooterTarget);
        telemetry.addData("shooter target velocity ", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity ", shooter.getActualVelocity());
        telemetry.addData("shooter hood pos ", shooter.getHoodPos());


        telemetry.addData("------------------",null);

        telemetry.addData("current pos ", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading ", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("saved pos ", String.format("X: %8.2f, Y: %8.2f", savedPose.getX(), savedPose.getY()));
        telemetry.addData("saved heading ", String.format("Heading: %.4f", savedPose.getHeading()));
        telemetry.addData("t value ", follower.getCurrentTValue());
        telemetry.addData("!follower.isBusy() || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2)", !follower.isBusy() || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2));
        telemetry.addData("slowmode ", slowMode);

        telemetry.addData("------------------",null);

        if (colorSensors.checkIfPurple(1) == true) {
            telemetry.addData("detecting purple, raw value: ", colorSensors.senseColorsHSV(1));
        }
        else if (colorSensors.checkIfPurple(2) == true) {
            telemetry.addData("detecting purple, raw value: ", colorSensors.senseColorsHSV(2));
        }
        if (colorSensors.checkIfGreen(1) == true) {
            telemetry.addData("detecting green, raw value: ", colorSensors.senseColorsHSV(1));
        }
        else if (colorSensors.checkIfGreen(2) == true) {
            telemetry.addData("detecting green, raw value: ", colorSensors.senseColorsHSV(2));
        }
        if (colorSensors.checkIfWhite(1) == true) {
            telemetry.addData("detecting white, raw value: ", colorSensors.senseColorsHSV(1));
        }
        else if (colorSensors.checkIfWhite(2) == true) {
            telemetry.addData("detecting white, raw value: ", colorSensors.senseColorsHSV(2));
        }

        timer.reset();
        telemetry.update();
        super.run();


    }
}