package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.ServoEx;

@Config
@TeleOp(name = "Hood pivot Position Tuning")
public class HoodPivotTuningOp extends OpMode {
    public static double pos = 0.5;
//    ServoEx gateLeft;
    ServoEx pivot;

    @Override
    public void init() {
//            gateLeft = new ServoEx(hardwareMap, "gateLeft");
            pivot = new ServoEx(hardwareMap, "pivot");
            pivot.setInverted(false);
    }

    @Override
    public void loop() {
//        gateLeft.set(pos);
        pivot.set(pos);
    }
}
