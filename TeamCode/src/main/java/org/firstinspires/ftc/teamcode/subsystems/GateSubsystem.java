package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

public class GateSubsystem extends SubsystemBase {
    ServoEx gate;
    AnalogInput gateEncoder;
    public final double UP = 0.55;
    public final double DOWN = 0.40;
    public final double UP_VOLTAGE = 1.6; //Volts
    public final double DOWN_VOLTAGE = 1.3; //Volts

    public GateSubsystem(final HardwareMap hMap) {
        gate = new ServoEx(hMap, "gate");
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
        if (Math.abs(gateEncoder.getVoltage() - UP_VOLTAGE) < 0.1 || Math.abs(gateEncoder.getVoltage() - DOWN_VOLTAGE) < 0.1) {
            return true;
        }
        else {
            return false;
        }
    }

}
