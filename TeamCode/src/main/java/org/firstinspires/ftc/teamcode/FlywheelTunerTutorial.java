package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
@TeleOp(name = "Prueba_shooter")
public class FlywheelTunerTutorial extends OpMode {

    public static double TARGET_VELOCITY = 1500;
    ServoEx hood;
    public static double kP = 0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0;

    public static double INTAKE_POWER = 1000;
    public DcMotorEx flywheelMotor, flywheelMotor2,intakeMotor;

    private boolean flywheelEnabled = false;
    private boolean intakeEnabled = false;

    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Flywheel
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Transfer");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updatePIDF();
    }

    private void updatePIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    @Override
    public void loop() {
        updatePIDF();

        boolean currentA = gamepad1.a;
        if (currentA && !lastA) {
            flywheelEnabled = !flywheelEnabled;
        }
        lastA = currentA;

        if (flywheelEnabled) {
            flywheelMotor.setVelocity(TARGET_VELOCITY);
            flywheelMotor2.setVelocity(TARGET_VELOCITY);
        } else {
            flywheelMotor.setVelocity(0);
            flywheelMotor2.setVelocity(0);
        }

        boolean currentY = gamepad1.y;
        if (currentY && !lastY) {
            intakeEnabled = !intakeEnabled;
        }
        lastY = currentY;

        if (intakeEnabled) {
            intakeMotor.setVelocity(INTAKE_POWER);
        } else {
            intakeMotor.setVelocity(0);
        }

        telemetry.addLine("--- FLYWHEEL ---");
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Vel 1", "%.1f", flywheelMotor.getVelocity());
        telemetry.addData("Current Vel 2", "%.1f", flywheelMotor2.getVelocity());
        telemetry.addData("Shooter Status", flywheelEnabled ? "ON (A)" : "OFF");

        telemetry.addLine("--- INTAKE ---");
        telemetry.addData("Intake Power", INTAKE_POWER);
        telemetry.addData("Intake Status", intakeEnabled ? "ON (Y)" : "OFF");

        telemetry.addData("kP", kP);
        telemetry.addData("kF", kF);
        telemetry.update();
    }
}