package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import Subsistemas.TurretSub;


@Config
@TeleOp(name = "TELEOPTHETA")
public class TeleOpODO extends OpMode {

    // Shooter
    private DcMotorEx flywheelMotor;
    private DcMotorEx flywheelMotor2;

    // Transferencia
    private DcMotorEx transferMotor;

    // Drivetrain
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // Servo
    private ServoEx servoTope;

    // Subsistema de la torreta
    private TurretSub turret;

    // PID del shooter
    private PIDFController shooterController;

    // Estados
    private boolean shooterRunning = false;
    private boolean transferRunning = false;

    // Shooter
    public static double targetVel = 750.0;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000525;


    @Override
    public void init() {

        /*
         * Limpiar el scheduler antes de crear subsistemas.
         * Debe hacerse antes de construir TurretSub.
         */
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().enable();

        initializeTurret();
        initializeShooter();
        initializeTransfer();
        initializeDrive();
        initializeServo();

        shooterController = new PIDFController(
                kP,
                kI,
                kD,
                0
        );

        telemetry.addLine("TELEOPBETA inicializado");
        telemetry.addLine("Esperando que Pinpoint esté READY");
        telemetry.update();
    }


    private void initializeTurret() {

        /*
         * Los nombres deben ser iguales a los nombres
         * usados en la configuración del robot.
         */
        turret = new TurretSub(
                hardwareMap,
                "TurretMotor",
                "pinpoint"
        );
    }


    private void initializeShooter() {

        flywheelMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        flywheelMotor2 = hardwareMap.get(
                DcMotorEx.class,
                "shooter2"
        );

        flywheelMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        flywheelMotor2.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        flywheelMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        flywheelMotor2.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        flywheelMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        flywheelMotor2.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }


    private void initializeTransfer() {

        transferMotor = hardwareMap.get(
                DcMotorEx.class,
                "Transfer"
        );

        transferMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        transferMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        transferMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }


    private void initializeDrive() {

        frontLeft = hardwareMap.get(
                DcMotorEx.class,
                "frontLeft"
        );

        frontRight = hardwareMap.get(
                DcMotorEx.class,
                "frontRight"
        );

        backLeft = hardwareMap.get(
                DcMotorEx.class,
                "backLeft"
        );

        backRight = hardwareMap.get(
                DcMotorEx.class,
                "backRight"
        );

        frontLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        backLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        frontRight.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        backRight.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeft.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        frontRight.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        backLeft.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        backRight.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }


    private void initializeServo() {
        servoTope = new ServoEx(
                hardwareMap,
                "ServoTope"
        );
    }


    @Override
    public void start() {

        turret.enable();

        shooterController.reset();

        shooterRunning = false;
        transferRunning = false;
    }


    @Override
    public void loop() {

        /*
         * Ejecuta TurretSub.periodic().
         * Si eliminas esta línea, la torreta no se actualizará.
         */
        CommandScheduler.getInstance().run();

        driveControl();
        servoControl();
        shooterControl();
        transferControl();
        turretControls();

        addShooterTelemetry();
        addTurretTelemetry();

        telemetry.update();
    }


    private void driveControl() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        double frontLeftPower =
                y + x + rotation;

        double frontRightPower =
                y - x - rotation;

        double backLeftPower =
                y - x + rotation;

        double backRightPower =
                y + x - rotation;

        double maximum = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeftPower),
                        Math.max(
                                Math.abs(frontRightPower),
                                Math.max(
                                        Math.abs(backLeftPower),
                                        Math.abs(backRightPower)
                                )
                        )
                )
        );

        frontLeft.setPower(
                frontLeftPower / maximum
        );

        frontRight.setPower(
                frontRightPower / maximum
        );

        backLeft.setPower(
                backLeftPower / maximum
        );

        backRight.setPower(
                backRightPower / maximum
        );
    }


    private void servoControl() {

        if (gamepad2.left_bumper) {
            servoTope.set(0.05);
        } else {
            servoTope.set(0.50);
        }
    }


    private void shooterControl() {

        if (gamepad2.aWasPressed()) {

            shooterRunning = !shooterRunning;

            /*
             * Evita conservar el error integral
             * de la ejecución anterior.
             */
            shooterController.reset();
        }

        if (!shooterRunning) {

            flywheelMotor.setPower(0.0);
            flywheelMotor2.setPower(0.0);

            return;
        }

        double flywheelVelocity1 =
                flywheelMotor.getVelocity();

        double flywheelVelocity2 =
                flywheelMotor2.getVelocity();

        double currentVelocity =
                (flywheelVelocity1 + flywheelVelocity2)
                        / 2.0;

        shooterController.setPIDF(
                kP,
                kI,
                kD,
                0
        );

        shooterController.setSetPoint(targetVel);

        double pidPower =
                shooterController.calculate(
                        currentVelocity
                );

        double feedforwardPower =
                kV * targetVel;

        double shooterPower =
                feedforwardPower + pidPower;

        shooterPower = clamp(
                shooterPower,
                -1.0,
                1.0
        );

        flywheelMotor.setPower(shooterPower);
        flywheelMotor2.setPower(shooterPower);
    }


    private void transferControl() {

        if (gamepad2.yWasPressed()) {
            transferRunning = !transferRunning;
        }

        if (transferRunning) {
            transferMotor.setPower(1.0);
        } else {
            transferMotor.setPower(0.0);
        }
    }


    private void turretControls() {

        /*
         * X:
         * activar o desactivar la corrección automática.
         */
        if (gamepad2.xWasPressed()) {
            turret.toggleEnabled();
        }

        /*
         * B:
         * poner el heading actual del robot en 0.
         */
        if (gamepad2.bWasPressed()) {
            turret.resetHeading();
        }

        /*
         * BACK:
         * reiniciar heading y encoder de la torreta.
         *
         * Úsalo solamente cuando la torreta esté
         * físicamente en su posición inicial de 90 grados.
         */
        if (gamepad2.backWasPressed()) {
            turret.resetAll();
            turret.enable();
        }
    }


    private void addShooterTelemetry() {

        double velocity1 =
                flywheelMotor.getVelocity();

        double velocity2 =
                flywheelMotor2.getVelocity();

        double averageVelocity =
                (velocity1 + velocity2) / 2.0;

        telemetry.addLine("----- SHOOTER -----");

        telemetry.addData(
                "Shooter activo",
                shooterRunning
        );

        telemetry.addData(
                "Velocidad 1",
                velocity1
        );

        telemetry.addData(
                "Velocidad 2",
                velocity2
        );

        telemetry.addData(
                "Velocidad promedio",
                averageVelocity
        );

        telemetry.addData(
                "Velocidad objetivo",
                targetVel
        );

        telemetry.addData(
                "Transfer activo",
                transferRunning
        );
    }


    private void addTurretTelemetry() {

        telemetry.addData("Robot X", turret.getRobotX());
        telemetry.addData("Robot Y", turret.getRobotY());

        telemetry.addData(
                "Distancia goal",
                turret.getGoalDistance()
        );

        telemetry.addData(
                "Bearing goal",
                turret.getGoalBearing()
        );

        telemetry.addData(
                "Bearing inicial",
                turret.getInitialGoalBearing()
        );


        telemetry.addData(
                "Heading",
                turret.getHeading()
        );

        telemetry.addData(
                "Ángulo deseado",
                turret.getDesiredAngle()
        );

        telemetry.addLine("----- TURRET -----");

        telemetry.addData(
                "Torreta activa",
                turret.isEnabled()
        );

        telemetry.addData(
                "Pinpoint status",
                turret.getPinpointStatus()
        );

        telemetry.addData(
                "Pinpoint frequency",
                turret.getPinpointFrequency()
        );

        telemetry.addData(
                "Robot heading",
                turret.getHeading()
        );

        telemetry.addData(
                "Heading velocity",
                turret.getHeadingVelocity()
        );

        telemetry.addData(
                "Ángulo actual",
                turret.getCurrentAngle()
        );

        telemetry.addData(
                "Ángulo deseado",
                turret.getDesiredAngle()
        );

        telemetry.addData(
                "Ángulo objetivo",
                turret.getTargetAngle()
        );

        telemetry.addData(
                "Ticks actuales",
                turret.getCurrentTicks()
        );

        telemetry.addData(
                "Ticks objetivo",
                turret.getTargetTicks()
        );

        telemetry.addData(
                "Error ticks",
                turret.getErrorTicks()
        );

        telemetry.addData(
                "Potencia torreta",
                turret.getAppliedPower()
        );

        telemetry.addData(
                "Ticks por vuelta",
                turret.getTicksPerTurretRev()
        );
    }


    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }


    @Override
    public void stop() {

        // Detener torreta
        if (turret != null) {
            turret.disable();
        }

        // Detener shooter
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0.0);
        }

        if (flywheelMotor2 != null) {
            flywheelMotor2.setPower(0.0);
        }

        // Detener transferencia
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }

        // Detener drivetrain
        if (frontLeft != null) {
            frontLeft.setPower(0.0);
        }

        if (frontRight != null) {
            frontRight.setPower(0.0);
        }

        if (backLeft != null) {
            backLeft.setPower(0.0);
        }

        if (backRight != null) {
            backRight.setPower(0.0);
        }

        shooterRunning = false;
        transferRunning = false;

        // Limpiar subsistemas y comandos registrados
        CommandScheduler.getInstance().reset();
    }
}