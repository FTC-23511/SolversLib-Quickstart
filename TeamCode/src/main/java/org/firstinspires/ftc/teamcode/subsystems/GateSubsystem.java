package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

public class GateSubsystem extends SubsystemBase {
//    ServoEx gateLeft;
    ServoEx gateRight;
    public double UP = 0.55;
    public double DOWN = 0.40;
    public GateSubsystem(final HardwareMap hMap) {
//        gateLeft = new ServoEx(hMap, "gateLeft");
        gateRight = new ServoEx(hMap, "gateRight");
//        gateLeft.setInverted(false);
        gateRight.setInverted(true);
    }
    public void up() {
//        gateLeft.set(UP);
        gateRight.set(UP);
    }
    public void down() {
//        gateLeft.set(DOWN);
        gateRight.set(DOWN);
    }
    public double getPosition() {
        return gateRight.getRawPosition();
    }
}
