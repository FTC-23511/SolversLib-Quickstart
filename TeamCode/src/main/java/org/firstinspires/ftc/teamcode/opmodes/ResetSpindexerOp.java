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

@TeleOp (name = "\uD83D\uDC80 Reset Spindexer", group = "OpModes")
public class ResetSpindexerOp extends CommandOpMode {
    private SpindexerSubsystem spindexer;
    private GamepadEx driver1;
    @Override
    public void initialize () {
        //systems and pedro
        driver1 = new GamepadEx(gamepad1);
        spindexer = new SpindexerSubsystem(hardwareMap);

        super.reset();
        register(spindexer);

        spindexer.DANGEROUS_RESETENCODER();
        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(spindexer::DANGEROUS_RESETENCODER)
        );

    }



    @Override
    public void run() {
        telemetry.addData("Press circle to reset spindexer.", ":)");
        telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());
        telemetry.update();
        super.run();

    }
}