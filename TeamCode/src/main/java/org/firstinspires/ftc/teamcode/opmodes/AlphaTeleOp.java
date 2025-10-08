package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@TeleOp (name = "Very cool teleop", group = "idk")
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
        follower = Constants.createFollower(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubSystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        super.reset();

        //pedro and gamepad wrapper
        follower.startTeleopDrive();
        driver1 = new GamepadEx(gamepad1);

        //command binding
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.INTAKING)));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new InstantCommand(() -> intake.setSpeed(IntakeSubsystem.IntakeState.STILL)));

        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(new InstantCommand(() -> spindexer.advanceSpindexer()));
        

    }



    @Override
    public void run() {
        follower.setTeleOpDrive(driver1.getLeftY(), driver1.getLeftX(), driver1.getRightX(), true);
        follower.update();


        super.run();
    }
}