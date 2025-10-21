package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@TeleOp (name = "Alpha Teleop", group = "OpModes")
public class AlphaTeleOp extends CommandOpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(0,0,0);

    private IntakeSubsystem intake;
    private ShooterSubSystem shooter;
    private SpindexerSubsystem spindexer;
    private LEDSubSystem led;

    public GamepadEx driver1;
    public GamepadEx driver2;

    @Override
    public void initialize () {
        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubSystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        led = new LEDSubSystem(hardwareMap);

        super.reset();
        register(intake, shooter, spindexer);


        //pedro and gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

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
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> shooter.setTargetVelocity(0))
        );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> shooter.setTargetVelocity(0))
        );
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(1300)));
        new Trigger(
                () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(new InstantCommand(() -> shooter.setTargetVelocity(-300)));


    }



    @Override
    public void run() {
        follower.setTeleOpDrive(driver1.getLeftY(), -driver1.getLeftX(), -driver1.getRightX(), true);
        follower.update() ;

        if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -50) {
            led.setColor(LEDSubSystem.LEDState.RED);
        }
        else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
            led.setColor(LEDSubSystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubSystem.LEDState.GREEN);
        }

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