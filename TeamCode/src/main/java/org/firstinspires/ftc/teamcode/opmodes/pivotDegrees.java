package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@TeleOp(name = "Pivot Degrees")
public class pivotDegrees extends OpMode{
    public GamepadEx driver2;
    private ShooterSubsystem shooter;

    @Override
    public void init() {
        driver2 = new GamepadEx(gamepad2);
        shooter = new ShooterSubsystem(hardwareMap);

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
    public void loop() {
        double pivotPosition = shooter.getPivotPosition();

        telemetry.addData("pivot current position", pivotPosition);
        telemetry.update();
    }
}
