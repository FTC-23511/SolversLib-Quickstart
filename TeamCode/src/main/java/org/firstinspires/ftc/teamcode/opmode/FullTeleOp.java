package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class FullTeleOp extends OpMode{
    public GamepadEx driver = new GamepadEx(gamepad1);
    public GamepadEx operator = new GamepadEx(gamepad2);

    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        CommandScheduler.getInstance().run();
    }
}
