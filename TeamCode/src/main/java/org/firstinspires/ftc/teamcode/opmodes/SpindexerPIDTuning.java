package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.controller.PIDController;

@Config
@TeleOp (name = "Spindexer pid tuning", group = " Tuning ")
public class SpindexerPIDTuning extends OpMode {
    private PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0;

    public static int target=0;

    private DcMotor spindexer;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        spindexer = hardwareMap.get(DcMotor.class, "spindexer");
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int spindexerPos = spindexer.getCurrentPosition();
        double pid = controller.calculate(spindexerPos, target);
        double ff = 0.0;

        double power = pid + ff;

        spindexer.setPower(power);
        telemetry.addData("pos ", spindexerPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
