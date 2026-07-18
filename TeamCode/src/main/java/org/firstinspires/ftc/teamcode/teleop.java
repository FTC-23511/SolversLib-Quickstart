package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import utilidades.imuEx;

@Config
@TeleOp(name = "TELEOPBETA")
public class teleop extends OpMode {

    public DcMotorEx flywheelMotor, flywheelMotor2;
    PIDFController shooterController;

    public static double targetVel = 1285;

    public static double shooterKP = 0.;
    public static double shooterKI = 0.0;
    public static double shooterKD = 0.0;
    public static double shooterKF = 0.00;

    public static double shooterTolerance = 20;


    DcMotorEx Transfer;
    ServoEx ServoTope, hood;
    MotorEx TurretMotor;
    PIDFController TurretController;
    imuEx imu;

    boolean shooterRunning = false;
    boolean transferRunning = false;

    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public static double turretKP = 0.03510;
    public static double turretKI = 0.0;
    public static double turretKD = 0.000;
    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 15.0;
    public static double turretExternalGearRatio = 4.8;
    public static double turretMaxPower = 0.50;
    public static double turretStartAngle = 90.0;
    public static double turretToleranceTicks = 1.5;
    public static double turretMinAngle = 0.0;
    public static double turretMaxAngle = 180.0;

    @Override
    public void init() {

        flywheelMotor  = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotor2.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooterController = new PIDFController(shooterKP, shooterKI, shooterKD, 0.000422);
        shooterController.setTolerance(shooterTolerance);

        Transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        ServoTope = new ServoEx(hardwareMap, "ServoTope");
        hood = new ServoEx(hardwareMap, "hood");


        // Turret
        TurretMotor = new MotorEx(hardwareMap, "TurretMotor");
        TurretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        TurretMotor.stopAndResetEncoder();
        TurretMotor.setRunMode(Motor.RunMode.RawPower);

        TurretController = new PIDFController(turretKP, turretKI, turretKD, 0);

        imu = new imuEx(hardwareMap, "imu");
        imu.init();

        Transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Drivetrain
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hood.setInverted(true);
        ServoTope.setInverted(true);
    }

    @Override
    public void loop() {
        hood.set(0.58);
        double heading = 0;

        // === TURRET ===
        double currentTurretTicks = TurretMotor.getCurrentPosition();
        double currentTurretAngle = turretTicksToDegrees(currentTurretTicks);
        double desiredTurretAngle = turretStartAngle + heading;
        double targetTurretAngle = findBestTurretTarget(desiredTurretAngle, currentTurretAngle);
        double targetTurretTicks = Math.round(degreesToTurretTicks(targetTurretAngle) * 10.0) / 10.0;

        TurretController.setPIDF(turretKP, turretKI, turretKD, 0);
        TurretController.setSetPoint(targetTurretTicks);
        double turretPower = TurretController.calculate(currentTurretTicks);
        turretPower = clamp(turretPower, -turretMaxPower, turretMaxPower);

        double turretErrorTicks = targetTurretTicks - currentTurretTicks;
        if (Math.abs(turretErrorTicks) <= turretToleranceTicks) {
            turretPower = 0;
        }
        if (currentTurretAngle <= turretMinAngle && turretPower < 0) turretPower = 0;
        if (currentTurretAngle >= turretMaxAngle && turretPower > 0) turretPower = 0;

        TurretMotor.set(turretPower);

        // === DRIVETRAIN ===
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);

        // === SERVO ===
        ServoTope.set(gamepad2.left_bumper ? 0 : 0.7);


        if (gamepad2.aWasPressed()) shooterRunning = !shooterRunning;

        if (shooterRunning) {
            // Actualizamos los coeficientes cada loop (para que el dashboard funcione en tiempo real)
            shooterController.setPIDF(shooterKP, shooterKI, shooterKD, shooterKF);
            shooterController.setSetPoint(targetVel);
            shooterController.setTolerance(shooterTolerance);

            double currentVelocity = (flywheelMotor.getVelocity() + flywheelMotor2.getVelocity()) / 2.0;

            double power = shooterController.calculate(currentVelocity);

            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);

            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Current Vel", currentVelocity);
            telemetry.addData("Shooter Error", shooterController.getPositionError());
            telemetry.addData("At Setpoint", shooterController.atSetPoint());
            telemetry.addData("Power", power);
        } else {
            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
            shooterController.reset();
        }

        if (gamepad2.yWasPressed()) transferRunning = !transferRunning;
        Transfer.setPower(transferRunning ? 0.15 : 0);

        telemetry.addData("Robot heading", heading);
        telemetry.addData("Current turret angle", currentTurretAngle);
        telemetry.addData("Target turret angle", targetTurretAngle);
        telemetry.addData("Turret power", turretPower);

        telemetry.update();
    }


    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTicksPerTurretRev() {
        return motorEncoderTicksPerRev * motorGearboxRatio * turretExternalGearRatio;
    }

    public double degreesToTurretTicks(double turretAngle) {
        return (turretAngle - turretStartAngle) * getTicksPerTurretRev() / 360.0;
    }

    public double turretTicksToDegrees(double ticks) {
        return turretStartAngle + ticks * 360.0 / getTicksPerTurretRev();
    }

    public double findBestTurretTarget(double desiredAngle, double currentAngle) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;
        boolean foundValid = false;

        for (int i = -64; i <= 64; i++) {
            double candidate = desiredAngle + 360.0 * i;
            if (candidate >= turretMinAngle && candidate <= turretMaxAngle) {
                double distance = Math.abs(candidate - currentAngle);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAngle = candidate;
                    foundValid = true;
                }
            }
        }
        if (foundValid) return bestAngle;
        return clamp(desiredAngle, turretMinAngle, turretMaxAngle);
    }
}