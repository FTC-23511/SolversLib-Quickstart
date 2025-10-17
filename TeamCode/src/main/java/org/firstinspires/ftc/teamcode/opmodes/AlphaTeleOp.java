package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@TeleOp (name = "Alpha Teleop", group = "OpModes")
public class AlphaTeleOp extends CommandOpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(0,0,0);

    private IntakeSubsystem intake;
    private ShooterSubSystem shooter;
    private SpindexerSubsystem spindexer;

    public GamepadEx driver1;

    @Override
    public void initialize () {
        //systems and pedro
//        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubSystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        super.reset();
        register(intake, shooter, spindexer);


        //pedro and gamepad wrapper
//        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);

        //command binding

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
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(0.8)));
        new Trigger(
                () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(0)));


    }



    @Override
    public void run() {
//        follower.setTeleOpDrive(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX(), true);
//        follower.update() ;




        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter pos", shooter.getShooterPosition());
        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.update();
        super.run();

    }
}