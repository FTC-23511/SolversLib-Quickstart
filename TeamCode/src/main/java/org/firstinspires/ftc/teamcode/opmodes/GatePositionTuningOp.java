package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

@Config
@TeleOp(name = "Gate Position Tuning")
public class GatePositionTuningOp extends OpMode {
    public static double pos = 0.5;
//    ServoEx gateLeft;
    ServoEx gateRight;

    @Override
    public void init() {
//            gateLeft = new ServoEx(hardwareMap, "gateLeft");
            gateRight = new ServoEx(hardwareMap, "gateRight");
//            gateLeft.setInverted(false);
            gateRight.setInverted(true);
    }

    @Override
    public void loop() {
//        gateLeft.set(pos);
        gateRight.set(pos);
    }
}
