package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

@Config
@TeleOp(name = "Gate Position Tuning")
public class GatePositionTuningOp extends OpMode {
    Servo gate;
    static double lowPos = 0.5;
    static double highPos = 0.6;
    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gate");
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            gate.setPosition(lowPos);
        }
        if (gamepad1.circle) {
            gate.setPosition(highPos);
        }
    }
}
