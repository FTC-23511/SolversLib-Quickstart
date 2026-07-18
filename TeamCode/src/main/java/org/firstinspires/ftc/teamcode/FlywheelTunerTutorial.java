package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
@TeleOp(name = "Prueba_shooter")
public class FlywheelTunerTutorial extends OpMode {

    public static double TARGET_VELOCITY = 1300;
// P = 0.0085
    // PID
    public static PIDFCoefficients SCoeffs = new PIDFCoefficients(0.0085, 0, 0, 0);

    // Feedforward
    public static double kV = 0.000455;   // Ajusta este valor

    public static double INTAKE_POWER = 1200;
    public static double Position = 0.15;

    public DcMotorEx flywheelMotor;
    public DcMotorEx flywheelMotor2;
    public DcMotorEx intakeMotor;
    public ServoEx hood;

    private PIDFController shooterPIDF;

    private boolean flywheelEnabled = false;
    private boolean intakeEnabled = false;

    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );


        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        hood = new ServoEx(hardwareMap, "hood");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Transfer");

        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        hood.setInverted(true);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterPIDF = new PIDFController(SCoeffs);
    }

    @Override
    public void loop() {
        hood.set(Position);

        shooterPIDF.setCoefficients(SCoeffs);

        boolean currentA = gamepad1.a;
        if (currentA && !lastA) {
            flywheelEnabled = !flywheelEnabled;
        }
        lastA = currentA;

        if (flywheelEnabled) {

            double currentVelocity = flywheelMotor2.getVelocity();

            shooterPIDF.setSetPoint(TARGET_VELOCITY);

            double power = (kV * TARGET_VELOCITY)
                    + shooterPIDF.calculate(currentVelocity);

            power = Math.max(-1.0, Math.min(1.0, power));

            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);

            telemetry.addData("Power", power);

        } else {

            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
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

        telemetry.addLine("------ SHOOTER ------");
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", flywheelMotor2.getVelocity());
        telemetry.addData("Power", flywheelMotor.getPower());
        telemetry.addData("Shooter", flywheelEnabled ? "ON" : "OFF");

        telemetry.addLine("------ PID ------");
        telemetry.addData("P", SCoeffs.p);
        telemetry.addData("I", SCoeffs.i);
        telemetry.addData("D", SCoeffs.d);
        telemetry.addData("kV", kV);

        telemetry.addLine("------ INTAKE ------");
        telemetry.addData("Intake", intakeEnabled ? "ON" : "OFF");

        telemetry.update();
    }
}