package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.Supplier;

@TeleOp (name = "Alpha Teleop", group = "OpModes")
public class AlphaTeleOp extends CommandOpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(0,0,0);
    public static Pose savedPose = new Pose(0,0,0);
    private Supplier<PathChain> pathChainSupplier;

    private IntakeSubsystem intake;
    private ShooterSubSystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSubSystem colorSensor;
    private LEDSubSystem led;

    public GamepadEx driver1;
    public GamepadEx driver2;

    private boolean manualControl = true;


    private void setSavedPose(Pose pose) {
        savedPose = pose;
        gamepad1.rumbleBlips(1);
    }
    private void goToSavedPose() {
        manualControl = false;
        follower.followPath(pathChainSupplier.get());
        gamepad1.rumbleBlips(3);
    }

    @Override
    public void initialize () {
        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubSystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorSensor = new ColorSubSystem(hardwareMap);
        led = new LEDSubSystem(hardwareMap);

        super.reset();
        register(intake, shooter, spindexer);



        //pedro and gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        pathChainSupplier = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, savedPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        //command binding

        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).toggleWhenActive(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.STILL))
        );
        driver1.getGamepadButton(GamepadKeys.Button.CROSS).toggleWhenActive(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.REVERSE)),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.STILL))
        );
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> spindexer.advanceSpindexer())
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> spindexer.reverseSpindexer())
        );
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> goToSavedPose())
        );
        driver1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> setSavedPose(follower.getPose()))
        );
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> shooter.setTargetVelocity(1300))
        );
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> shooter.setTargetVelocity(-300))
        );
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(+0)));
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(-0)));


    }



    @Override
    public void run() {
        if (manualControl) {
            follower.setTeleOpDrive(driver1.getLeftY(), -driver1.getLeftX(), -driver1.getRightX(), true);
        } else {
            if (!follower.isBusy() || (gamepad1.touchpad_finger_1 && gamepad1.touchpad_finger_2)) {
                manualControl = true;
            }
        }
        follower.update();
        if (shooter.getActualVelocity() > 300) { //shooting mode
            if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -50) {
                led.setColor(LEDSubSystem.LEDState.RED);
            }
            else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
                led.setColor(LEDSubSystem.LEDState.BLUE);
            }
            else {
                led.setColor(LEDSubSystem.LEDState.GREEN);
            }
        }
        else { //intaking mode
            if (colorSensor.checkIfGreen()) {
                led.setColor(LEDSubSystem.LEDState.VIOLET);
            }
            else if (colorSensor.checkIfPurple()) {
                led.setColor(LEDSubSystem.LEDState.GREEN);
            }
            else {
                led.setColor(LEDSubSystem.LEDState.WHITE); //anything else besides green or purple
            }
        }

        telemetry.addData("current pos", follower.getPose().toString());
        telemetry.addData("saved pos", savedPose.toString());

        telemetry.addData("------------------",null);


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