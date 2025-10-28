package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@Config
@TeleOp(name = "Shooter Tuning Op", group = "OpModes")
public class ShooterTuningOp extends CommandOpMode {

    private Follower follower;
    public static Pose startingPose = new Pose(0, 0, 0);

    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;
    private Motor shooter;

    // Dashboard-tunable PIDF constants
    public static double kP = 0.004;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00055;

    // Velocity targets (tunable)
    public static double targetVelocity = 0.0;

    private PIDFController flywheelController;

    public GamepadEx driver1;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize systems
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        shooter = new Motor(hardwareMap, "shooter", Motor.GoBILDA.RPM_312);

        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter.set(0.0);

        // Create PIDF controller
        flywheelController = new PIDFController(kP, kI, kD, kF);

        super.reset();
        register(intake, spindexer);

        // Gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);

        // Command bindings
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE).toggleWhenActive(
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)),
                new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.STILL))
        );

        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> spindexer.advanceSpindexer())
        );

        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> spindexer.reverseSpindexer())
        );

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> targetVelocity = 1500.0)
        );

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> targetVelocity = 0.0)
        );
    }

    @Override
    public void run() {
        // Update follower drive
        follower.setTeleOpDrive(driver1.getLeftY(), -driver1.getLeftX(), -driver1.getRightX(), true);
        follower.update();

        // Update PIDF controller
        flywheelController.setPIDF(kP, kI, kD, kF);
        flywheelController.setSetPoint(targetVelocity * 275); // same scaling as ShooterSubSystem

        double shooterVelocity = shooter.getCorrectedVelocity();
        double output = flywheelController.calculate(shooterVelocity);
        shooter.set(output);

        // Telemetry
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", shooterVelocity);
        telemetry.addData("Controller Output", output);
        telemetry.addData("PIDF", "P: %.5f I: %.5f D: %.5f F: %.5f", kP, kI, kD, kF);

        telemetry.addData("Spindexer Output", spindexer.getOutput());
        telemetry.addData("Spindexer Setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("Spindexer Pos", spindexer.getCurrentPosition());
        telemetry.update();

        super.run();
    }
}
