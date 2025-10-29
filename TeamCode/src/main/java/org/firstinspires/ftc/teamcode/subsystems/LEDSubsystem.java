package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Servo led;
    public enum LEDState {
        OFF, RED, ORANGE, YELLOW, SAGE, GREEN, AZURE, BLUE, INDIGO, VIOLET, WHITE;
    }
    public LEDState ledstate = LEDState.OFF;
    public LEDSubsystem(final HardwareMap hMap) {
        led = hMap.get(Servo.class, "led");
    }

    public void setColor(LEDState i) {
        if (i == LEDState.OFF) {
            led.setPosition(0.0);
        }
        else if (i == LEDState.RED) {
            led.setPosition(0.277);
        }
        else if (i == LEDState.ORANGE) {
            led.setPosition(0.333);
        }
        else if (i == LEDState.YELLOW) {
            led.setPosition(0.388);
        }
        else if (i == LEDState.SAGE) {
            led.setPosition(0.444);
        }
        else if (i == LEDState.GREEN) {
            led.setPosition(0.500);
        }
        else if (i == LEDState.AZURE) {
            led.setPosition(0.555);
        }
        else if (i == LEDState.BLUE) {
            led.setPosition(0.611);
        }
        else if (i == LEDState.INDIGO) {
            led.setPosition(0.666);
        }
        else if (i == LEDState.VIOLET) {
            led.setPosition(0.722);
        }
        else if (i == LEDState.WHITE) {
            led.setPosition(1.0);
        }
        ledstate = i;
    }

}
