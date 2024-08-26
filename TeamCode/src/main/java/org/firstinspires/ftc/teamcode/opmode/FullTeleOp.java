package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class FullTeleOp extends OpMode{
    public GamepadEx driver = new GamepadEx(gamepad1);
    public GamepadEx operator = new GamepadEx(gamepad2);

    private final Robot robot = Robot.getInstance();

    @Override
    public void init() {
        // Also initializes individual subsystems and imu within the function
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();

        CommandScheduler.getInstance().run();
    }
}
