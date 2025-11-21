package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.ServoEx;

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
