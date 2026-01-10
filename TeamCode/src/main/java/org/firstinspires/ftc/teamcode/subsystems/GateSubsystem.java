package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

public class GateSubsystem extends SubsystemBase {
    ServoEx gate;
    AnalogInput gateEncoder;
    public final double UP = 0.865;
    public final double DOWN = 0.73;
    public final double UP_VOLTAGE = 2.737; //Volts
    public final double DOWN_VOLTAGE = 2.334; //Volts

    public GateSubsystem(final HardwareMap hMap) {
        gate = new ServoEx(hMap, "gate");
        gateEncoder = hMap.get(AnalogInput.class, "gateEncoder");
        gate.setInverted(true);
    }
    public enum GateState {
        UP, DOWN;
    }
    public GateState gateState;
    public void up() {
        gate.set(UP);
        gateState = GateState.UP;
    }
    public void down() {
        gate.set(DOWN);
        gateState = GateState.DOWN;
    }
    public double getEncoderVoltage() {
        return gateEncoder.getVoltage();
    }
    public boolean isAtTarget() {
        //is gate at position
        //add wait for gate finish command
        return Math.abs(gateEncoder.getVoltage() - UP_VOLTAGE) < 0.1 || Math.abs(gateEncoder.getVoltage() - DOWN_VOLTAGE) < 0.1;
    }

}
