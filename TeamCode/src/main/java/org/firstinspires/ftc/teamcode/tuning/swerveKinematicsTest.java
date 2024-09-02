package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DriveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.MAXIMUM_MODULE_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.STARTING_HEADING;
import static org.firstinspires.ftc.teamcode.hardware.Globals.STRAFE_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.hardware.Globals.TURN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.hardware.Globals.driveMode;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class swerveKinematicsTest extends CommandOpMode {
    public GamepadEx driver;
    public ElapsedTime timer = null;

    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;
        driveMode = DriveMode.FIELD_CENTRIC;
        STARTING_HEADING = 0;

        // Resets command scheduler
        super.reset();

        robot.init(hardwareMap);

        register(robot.swerveDrivetrain);

        driver = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            // DO NOT REMOVE! Gets the IMU readings on separate thread
            robot.startIMUThread(this);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            robot.resetIMU();
        }

        double speedMultiplier = (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.8) + 0.2;

        // vX for this is moving forward, vY is moving left, driver.getLeftY() has to be reversed because up is down by default
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds (
                driver.getLeftY() * STRAFE_MULTIPLIER * -speedMultiplier,
                driver.getLeftX() * STRAFE_MULTIPLIER * speedMultiplier,
                driver.getRightX() * TURN_MULTIPLIER * speedMultiplier,
                new Rotation2d(robot.getAngle())
        );

        SwerveModuleState[] moduleStates = robot.swerveDrivetrain.update(chassisSpeeds);

        telemetry.addData("front left module motor power", moduleStates[0].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        telemetry.addData("front right module motor power", moduleStates[1].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        telemetry.addData("back left module motor power", moduleStates[2].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);
        telemetry.addData("back right module motor power", moduleStates[3].speedMetersPerSecond/MAXIMUM_MODULE_SPEED);

        telemetry.addData("front left module angle", moduleStates[0].angle.getRadians());
        telemetry.addData("front right module angle", moduleStates[1].angle.getRadians());
        telemetry.addData("back left module angle", moduleStates[2].angle.getRadians());
        telemetry.addData("back right module angle", moduleStates[3].angle.getRadians());

        telemetry.addData("ChassisSpeeds rad/sec", chassisSpeeds.omegaRadiansPerSecond);
        telemetry.addData("ChassisSpeeds vx m/s", chassisSpeeds.vxMetersPerSecond);
        telemetry.addData("ChassisSpeeds vy m/s", chassisSpeeds.vyMetersPerSecond);
        telemetry.addData("robot angle", robot.getAngle());
        telemetry.update();
    }
}
