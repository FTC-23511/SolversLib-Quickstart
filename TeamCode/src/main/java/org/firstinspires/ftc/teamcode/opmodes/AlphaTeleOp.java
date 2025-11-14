package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.Motifs.*;

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
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.RobotConstants.*;
import org.firstinspires.ftc.teamcode.commands.ScanAndUpdateBallsCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

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

    //gamepads
    public GamepadEx driver1;
    public GamepadEx driver2;

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
    double closeShooterTarget = 1100;

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
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        super.reset();
        lastVoltageCheck.reset();
        register(intake, shooter, spindexer);

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
//        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
//                new InstantCommand(() -> {
//                    spindexer.advanceSpindexer();
//                })
//        );
//        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
//                new InstantCommand(() -> {
//                    spindexer.reverseSpindexer();
//                })
//        );
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
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));
        new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whileActiveContinuous(new InstantCommand(() -> slowMode = true))
                .whenInactive(new InstantCommand(() -> slowMode = false));
        //Driver 2
        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    gate.gateUp();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    gate.gateDown();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.OPTIONS).whenPressed(
                new InstantCommand(() -> {
                    shooter.increasePivotPosition(1);
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    shooter.decreasePivotPosition(1);
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
        /*
        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    spindexer.moveSpindexerBy(60);
                    spindexerAdjustmentCount += 60;
                    gamepad2.rumbleBlips(1);
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    spindexer.moveSpindexerBy(-60);
                    spindexerAdjustmentCount -= 60;
                    gamepad2.rumbleBlips(1);
                })
        );*/
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
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed( //close close distance
                new InstantCommand(() -> {
                    shooter.setTargetVelocity(1100);
                })
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) //far close distance
                    .whileActiveContinuous(new InstantCommand(() -> {
                        shooter.setTargetVelocity(closeShooterTarget);
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
                            shooter.setTargetVelocity(-300);
                        })
                );


    }



    @Override
    public void run() {
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
            if (
                    (Math.abs(follower.getPose().getHeading() - savedPose.getHeading()) < 0.04
                    && Math.abs(follower.getPose().getX() - savedPose.getX()) < 1
                    && Math.abs(follower.getPose().getY() - savedPose.getY()) < 1)
                    || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2)) {
                manualControl = true;
                follower.startTeleopDrive();
            }
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
            if (colorSensors.checkIfGreen(colorSensors.senseColorsHSV(1))) {
                led.setColor(LEDSubsystem.LEDState.GREEN);
            }
            else if (colorSensors.checkIfPurple(colorSensors.senseColorsHSV(1))) {
                led.setColor(LEDSubsystem.LEDState.VIOLET);
            }
            else if (colorSensors.checkIfWhite(colorSensors.senseColorsHSV(1))){
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


        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("spindexer tick adjustment degrees", spindexerAdjustmentCount);

        telemetry.addData("------------------",null);

        telemetry.addData("shooter close amount", closeShooterTarget);
        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());

        telemetry.addData("------------------",null);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("saved pos", String.format("X: %8.2f, Y: %8.2f", savedPose.getX(), savedPose.getY()));
        telemetry.addData("saved heading", String.format("Heading: %.4f", savedPose.getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("!follower.isBusy() || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2)", !follower.isBusy() || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2));
        telemetry.addData("slowmode", slowMode);
        telemetry.addData("------------------",null);

        timer.reset();
        telemetry.update();
        super.run();


    }
}