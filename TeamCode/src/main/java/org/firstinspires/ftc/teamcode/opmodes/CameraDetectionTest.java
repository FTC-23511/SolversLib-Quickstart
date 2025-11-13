package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;

@TeleOp(name = "Camera Detection Test")
public class CameraDetectionTest extends CommandOpMode {
    private CameraSubsystem camera;
    @Override
    public void initialize() {
        camera = new CameraSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        camera.detectMotif();
    }
}
