package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

@Config
@TeleOp(name = "Gate Position Tuning")
public class gatePositionTuning extends CommandOpMode {
    public GamepadEx driver2;
    private GateSubsystem gate;
    @Override
    public void initialize() {
        driver2 = new GamepadEx(gamepad2);
        gate = new GateSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    gate.gateUp();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    gate.gateDown();
                })
        );
    }

    @Override
    public void run() {
        double gatePosition = gate.getGatePosition();

        telemetry.addData("gate position: ", gatePosition);
        telemetry.update();
    }
}
