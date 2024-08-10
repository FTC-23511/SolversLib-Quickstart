package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class risingEdge {
    public boolean checkButton(Gamepad gamepad, String button) {
        String buttons = String.valueOf(gamepad).substring(75).substring(1);
        return buttons.contains(button);
    }
}