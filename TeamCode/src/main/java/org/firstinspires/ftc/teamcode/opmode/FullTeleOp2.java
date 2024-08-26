package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.hardware.Globals.STRAFE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.hardware.Globals.TURN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;


public class FullTeleOp2 extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public ElapsedTime timer = null;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        opModeType = Globals.OpModeType.TELEOP;
        driveMode = Globals.DriveMode.FIELD_CENTRIC;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        register(robot.deposit, robot.intake, robot.swerveDrivetrain);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void run() {
        // Only run first time, but not in init because IMU thread would start polling?
        // TO-DO: Need to check if this is needed or if it is safe to put in initialize()
        if (timer == null) {
            timer = new ElapsedTime();
            // DO NOT REMOVE! Gets the IMU readings on separate thread
            robot.startIMUThread(this);
        }
        // Endgame/hang rumble after 110 seconds to notify driver to hang
        else if ((timer.seconds() > 110) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        // Runs the command scheduler and performs all the periodic() functions of each subsystem
        super.run();

        // Driving stuff
        // Minimum power of 0.2 and scale trigger value by remainder
        // Value to scale power to drivetrain based on driver trigger
        double speedMultiplier = (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.8) + 0.2;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driver.getLeftX() * STRAFE_MULTIPLIER * speedMultiplier,
                driver.getLeftY() * STRAFE_MULTIPLIER * speedMultiplier,
                driver.getRightX() * TURN_MULTIPLIER * speedMultiplier,
                new Rotation2d(robot.getAngle())
                );

        robot.swerveDrivetrain.update(chassisSpeeds);

        // Driver buttons
        

        // Operator buttons


        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }



}
