package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Gamepad;

public class risingEdge {
    // Example Usage for gamepad1 right bumper: risingEdge.checkButton(currentGamepad1, "right_bumper")
    public static boolean checkButton(Gamepad gamepad, String button) {
        try {
            String buttons = String.valueOf(gamepad).substring(75).substring(1);
            return !buttons.contains(button);
        }
        catch (Exception ignored) {
            return (true);
        }
    }
}