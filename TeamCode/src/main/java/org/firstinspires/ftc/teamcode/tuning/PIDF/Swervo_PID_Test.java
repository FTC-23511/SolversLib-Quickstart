package org.firstinspires.ftc.teamcode.tuning.PIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Photon
@Config
@TeleOp
public class Swervo_PID_Test extends OpMode {
    public static int setPoint = 0;

    private final Robot robot = Robot.getInstance();

    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        timer.reset();

        robot.fL.read();
        robot.fL.update(setPoint, 0);

        robot.ControlHub.clearBulkCache();

        telemetry.addData("encoder position", robot.frontLeftServo.getPosition());
        telemetry.addData("servoPower", robot.frontLeftServo.getPower());
        telemetry.addData("setPoint", setPoint);
        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }
}