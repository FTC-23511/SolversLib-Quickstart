package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Pivot Degrees")
public class pivotDegrees extends CommandOpMode{
    public GamepadEx driver2;
    private ShooterSubsystem shooter;

    @Override
    public void initialize() {
        driver2 = new GamepadEx(gamepad2);
        shooter = new ShooterSubsystem(hardwareMap);
        //when starting, it will try its hardest to get to position of 0 ticks
        //check telemetry to see pivot actual position and use buttons to adjust to 0 degrees flat
        //set that position as the offset so every time subsystem initializes pivot to that position

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    shooter.increasePivotPosition(0.001);
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    shooter.decreasePivotPosition(0.001);
                })
        );
    }

    @Override
    public void run() {
        double pivotPosition = shooter.getPivotPosition();

        telemetry.addData("pivot current position: ", pivotPosition);
        telemetry.update();
    }
}
