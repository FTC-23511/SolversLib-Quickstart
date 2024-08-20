package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.subsystem.System.checkButton;
import static org.firstinspires.ftc.teamcode.subsystem.System.round;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
<<<<<<< Updated upstream
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

//@Config
//@TeleOp
public class wristTester extends OpMode {
    public static double servoPos = 0.5;
    public static boolean USE_DASHBOARD = false;

    Deposit deposit = new Deposit(hardwareMap);
    Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit.wrist.setPosition(servoPos);
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            deposit.wrist.setPosition(servoPos);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            servoPos += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            servoPos -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            deposit.wrist.setPosition(servoPos);
        }

        servoPos = Math.max(Math.min(servoPos, 1), 0);

        currentGamepad1.copy(gamepad1);

        telemetry.addData("wristServo getPosition", round(deposit.wrist.getPosition(), 2));
        telemetry.addData("wristServoPos",round(servoPos, 2));
        telemetry.update();
=======
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
@TeleOp
public class wristTester extends LinearOpMode {
    public static boolean USE_DASHBOARD = false;
    public static double WRIST_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();

        robotHardware.wrist.setPosition(WRIST_POS);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (USE_DASHBOARD){
                robotHardware.wrist.setPosition(WRIST_POS);
            } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
                WRIST_POS += 0.01;
            } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
                WRIST_POS -= 0.01;
            }

            if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
                robotHardware.wrist.setPosition(WRIST_POS);
            }

            WRIST_POS = Math.max(Math.min(WRIST_POS, 1), 0);

            currentGamepad1.copy(gamepad1);

            telemetry.addData("wrist getPosition", round(robotHardware.wrist.getPosition(), 2));
            telemetry.addData("wristPos", round(WRIST_POS, 2));
            telemetry.update();
        }
>>>>>>> Stashed changes
    }
}