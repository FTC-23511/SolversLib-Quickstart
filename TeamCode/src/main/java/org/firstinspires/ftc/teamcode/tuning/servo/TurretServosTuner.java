package org.firstinspires.ftc.teamcode.tuning.servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.globals.Constants;
import org.firstinspires.ftc.teamcode.globals.Robot;

@Config
@TeleOp
public class TurretServosTuner extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;

    public static double LEFT_SERVO_POS = 0.0;
    public static double RIGHT_SERVO_POS = 0.0;
    public static double BOTH_SERVO_POS = 0.0;

    public static boolean MOVE_BOTH = false;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

    private final Robot robot = Robot.getInstance();
    
    @Override
    public void initialize() {
        // Must have for all opModes
        Constants.OP_MODE_TYPE = Constants.OpModeType.TELEOP;

        // Resets the command scheduler
        super.reset();

        // Initialize the robot (which also registers subsystems, configures CommandScheduler, etc.)
        robot.init(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver controls
        // Reset heading
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> robot.drive.setPose(new Pose2d()))
        );
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();
            timer = new ElapsedTime();
        }

        BOTH_SERVO_POS = Range.clip(BOTH_SERVO_POS, 0.0, 1.0);
        LEFT_SERVO_POS = Range.clip(LEFT_SERVO_POS, 0.0, 1.0);
        RIGHT_SERVO_POS = Range.clip(RIGHT_SERVO_POS, 0.0, 1.0);

        if (MOVE_BOTH) {
            robot.leftTurretServo.set(BOTH_SERVO_POS);
            robot.rightTurretServo.set(BOTH_SERVO_POS);
        } else {
            robot.leftTurretServo.set(LEFT_SERVO_POS);
            robot.rightTurretServo.set(RIGHT_SERVO_POS);
        }

        telemetryData.addData("Loop Time", timer.milliseconds());
        timer.reset();

        telemetryData.addData("LEFT_SERVO_POS", LEFT_SERVO_POS);
        telemetryData.addData("RIGHT_SERVO_POS", RIGHT_SERVO_POS);

        // DO NOT REMOVE ANY LINES BELOW! Runs the command scheduler and updates telemetry
        super.run();
        telemetryData.update();
    }
}