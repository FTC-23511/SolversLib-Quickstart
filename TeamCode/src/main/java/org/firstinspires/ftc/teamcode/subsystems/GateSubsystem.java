package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

public class GateSubsystem extends SubsystemBase {
    ServoEx gate;
    public double UP = 0.8;
    public double DOWN = 0.2;
    public GateSubsystem(final HardwareMap hMap) {
        gate = new ServoEx(hMap, "gate");
        gate.setInverted(false);
    }
    public void up() {
        gate.set(UP);
    }
    public void down() {
        gate.set(DOWN);
    }
    public double getPosition() {
        return gate.getRawPosition();
    }
}
