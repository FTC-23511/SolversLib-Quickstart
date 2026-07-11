package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import Subsistemas.TurretSub;

@Configurable
@TeleOp(name = "TELEOPTHETA")
public class TeleOpODO extends OpMode {

    // ================= PANELS =================

    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();

    /**
     * Activa la telemetría detallada de los gamepads en Panels.
     * Puede desactivarse desde Configurables para reducir tráfico.
     */
    public static boolean showPanelsGamepadDebug = true;

    // ================= HARDWARE =================

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

    // ================= ESTADOS =================

    private boolean shooterRunning = false;
    private boolean transferRunning = false;

    // ================= CONFIGURABLES =================

    public static double targetVel = 750.0;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000525;

    public static double servoMin = 0.05;
    public static double servoMax = 0.5;


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

        telemetry.addLine("TELEOPTHETA inicializado");
        telemetry.addLine("Esperando que Pinpoint esté READY");

        panelsTelemetry.debug("TELEOPTHETA inicializado");
        panelsTelemetry.debug("Esperando que Pinpoint esté READY");
        panelsTelemetry.update(telemetry);
    }


    private void initializeTurret() {

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
                DcMotorSimple.Direction.REVERSE
        );

        backRight.setDirection(
                DcMotorSimple.Direction.REVERSE
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
         * Combina el gamepad físico de FTC con el gamepad remoto de Panels.
         * A partir de aquí se deben usar g1 y g2, no gamepad1 y gamepad2.
         */
        Gamepad g1 = PanelsGamepad.INSTANCE
                .getFirstManager()
                .asCombinedFTCGamepad(gamepad1);

        Gamepad g2 = PanelsGamepad.INSTANCE
                .getSecondManager()
                .asCombinedFTCGamepad(gamepad2);

        /*
         * Ejecuta TurretSub.periodic().
         * Si eliminas esta línea, la torreta no se actualizará.
         */
        CommandScheduler.getInstance().run();

        driveControl(g1);
        servoControl(g2);
        shooterControl(g2);
        transferControl(g2);
        turretControls(g2);

        addShooterTelemetry();
        addTurretTelemetry();
        addPanelsRobotTelemetry();

        if (showPanelsGamepadDebug) {
            addPanelsGamepadTelemetry("GAMEPAD 1", g1);
            addPanelsGamepadTelemetry("GAMEPAD 2", g2);
        }

        /*
         * Actualiza tanto Panels como la telemetría normal del Driver Station.
         * No hace falta llamar telemetry.update() por separado.
         */
        panelsTelemetry.update(telemetry);
    }


    private void driveControl(Gamepad g1) {

        double y = -g1.left_stick_y;
        double x = g1.left_stick_x * 1.05;
        double rotation = g1.right_stick_x;

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


    private void servoControl(Gamepad g2) {

        if (g2.left_bumper) {
            servoTope.set(servoMin);
        } else {
            servoTope.set(servoMax);
        }
    }


    private void shooterControl(Gamepad g2) {

        if (g2.aWasPressed()) {

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


    private void transferControl(Gamepad g2) {

        if (g2.yWasPressed()) {
            transferRunning = !transferRunning;
        }

        if (transferRunning) {
            transferMotor.setPower(1.0);
        } else {
            transferMotor.setPower(0.0);
        }
    }


    private void turretControls(Gamepad g2) {

        /*
         * X:
         * activar o desactivar la corrección automática.
         */
        if (g2.xWasPressed()) {
            turret.toggleEnabled();
        }

        /*
         * B:
         * poner el heading actual del robot en 0.
         */
        if (g2.bWasPressed()) {
            turret.resetHeading();
        }

        /*
         * BACK:
         * reiniciar heading y encoder de la torreta.
         *
         * Úsalo solamente cuando la torreta esté
         * físicamente en su posición inicial de 90 grados.
         */
        if (g2.backWasPressed()) {
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

        telemetry.addLine("----- TURRET -----");

        telemetry.addData(
                "Robot X",
                turret.getRobotX()
        );

        telemetry.addData(
                "Robot Y",
                turret.getRobotY()
        );

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


    private void addPanelsRobotTelemetry() {

        panelsTelemetry.debug("==== ROBOT ====");
        panelsTelemetry.debug("Shooter activo: " + shooterRunning);
        panelsTelemetry.debug("Transfer activo: " + transferRunning);
        panelsTelemetry.debug("Torreta activa: " + turret.isEnabled());
        panelsTelemetry.debug("Robot X: " + turret.getRobotX());
        panelsTelemetry.debug("Robot Y: " + turret.getRobotY());
        panelsTelemetry.debug("Heading: " + turret.getHeading());
        panelsTelemetry.debug("Ángulo actual: " + turret.getCurrentAngle());
        panelsTelemetry.debug("Ángulo objetivo: " + turret.getTargetAngle());
        panelsTelemetry.debug("Error ticks: " + turret.getErrorTicks());
        panelsTelemetry.debug("Potencia torreta: " + turret.getAppliedPower());
    }


    private void addPanelsGamepadTelemetry(
            String name,
            Gamepad gamepad
    ) {

        panelsTelemetry.debug("==== " + name + " ====");

        panelsTelemetry.debug("A: " + gamepad.a);
        panelsTelemetry.debug("B: " + gamepad.b);
        panelsTelemetry.debug("X: " + gamepad.x);
        panelsTelemetry.debug("Y: " + gamepad.y);

        panelsTelemetry.debug("DPad Up: " + gamepad.dpad_up);
        panelsTelemetry.debug("DPad Down: " + gamepad.dpad_down);
        panelsTelemetry.debug("DPad Left: " + gamepad.dpad_left);
        panelsTelemetry.debug("DPad Right: " + gamepad.dpad_right);

        panelsTelemetry.debug("Left Bumper: " + gamepad.left_bumper);
        panelsTelemetry.debug("Right Bumper: " + gamepad.right_bumper);

        panelsTelemetry.debug("Left Trigger: " + gamepad.left_trigger);
        panelsTelemetry.debug("Right Trigger: " + gamepad.right_trigger);

        panelsTelemetry.debug("Start / Options: " + gamepad.options);
        panelsTelemetry.debug("Back / Share: " + gamepad.back);
        panelsTelemetry.debug("Guide / PS: " + gamepad.guide);
        panelsTelemetry.debug("Touchpad: " + gamepad.touchpad);

        panelsTelemetry.debug(
                "Left Stick Button: " + gamepad.left_stick_button
        );

        panelsTelemetry.debug(
                "Right Stick Button: " + gamepad.right_stick_button
        );

        panelsTelemetry.debug("Left Stick X: " + gamepad.left_stick_x);
        panelsTelemetry.debug("Left Stick Y: " + gamepad.left_stick_y);
        panelsTelemetry.debug("Right Stick X: " + gamepad.right_stick_x);
        panelsTelemetry.debug("Right Stick Y: " + gamepad.right_stick_y);
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

        panelsTelemetry.debug("TELEOPTHETA detenido");
        panelsTelemetry.update(telemetry);

        // Limpiar subsistemas y comandos registrados
        CommandScheduler.getInstance().reset();
    }
}