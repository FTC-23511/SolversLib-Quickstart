package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import java.util.List;

import Subsistemas.LauncherSub;
import Subsistemas.TurretSub;

@Configurable
@TeleOp(name = "TELEOPTHETA")
public class TeleOpODO extends OpMode {

    // ================= PANELS =================

    private final TelemetryManager panelsTelemetry =
            PanelsTelemetry.INSTANCE.getTelemetry();


    public static boolean showPanelsGamepadDebug = false;


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

    private LauncherSub launcher;
    private boolean transferRunning;


    // ================= ESTADOS =================

    public static double servoMin = 0.50;
    public static double servoMax = 0.02;







    @Override
    public void init() {

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(
                LynxModule.BulkCachingMode.AUTO));

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().enable();

        initializeTurret();
        initializeShooter();
        initializeTransfer();
        initializeDrive();
        initializeServo();



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
        // flywheel constructor
        launcher = new LauncherSub(hardwareMap, "shooter1","shooter2" ,"hood" );

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
        launcher.setGoalDistance(turret.getGoalDistance());

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
        if (g2.aWasPressed()){
            launcher.toggleShooter();
        }
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

        if (g2.xWasPressed()) {
            turret.toggleEnabled();
        }

    }


    private void addShooterTelemetry() {
        telemetry.addData("Shooter Running", launcher.isShooterRunning());

        telemetry.addData("Motor RPM", launcher.getMotorRPM());
        telemetry.addData("Shooter RPM", launcher.getShooterRPM());
        telemetry.addData("Target RPM", launcher.getTargetRPM());
        telemetry.addData("RPM Error", launcher.getRPMError());

        telemetry.addData("Flywheel Power", launcher.getFlywheelPower());

        telemetry.addData("Hood Angle", launcher.getHoodAngle());
        telemetry.addData("Hood Servo Pos", launcher.getHoodServoPosition());







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

        panelsTelemetry.debug("Transfer activo: " + transferRunning);
        panelsTelemetry.debug("Torreta activa: " + turret.isEnabled());
        panelsTelemetry.debug("Robot X: " + turret.getRobotX());
        panelsTelemetry.debug("Robot Y: " + turret.getRobotY());
        panelsTelemetry.debug("Heading: " + turret.getHeading());
        panelsTelemetry.debug("Ángulo actual: " + turret.getCurrentAngle());
        panelsTelemetry.debug("Ángulo objetivo: " + turret.getTargetAngle());
        panelsTelemetry.debug("Error ticks: " + turret.getErrorTicks());
        panelsTelemetry.debug("Potencia torreta: " + turret.getAppliedPower());
        panelsTelemetry.debug("motor RPM " + launcher.getMotorRPM());
        panelsTelemetry.debug("Shooter RPM " + launcher.getShooterRPM());
        panelsTelemetry.debug("Target RPM " + launcher.getTargetRPM());
        panelsTelemetry.debug("Erro RPM" + launcher.getRPMError());
        panelsTelemetry.debug("shooting Angle" + launcher.getHoodAngle());
        panelsTelemetry.debug("Servo Angle" + launcher.getHoodServoRawPosition());
        panelsTelemetry.debug("motor Power" + launcher.getFlywheelPower());


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



    @Override
    public void stop() {

        // Detener torreta
        if (turret != null) {
            turret.disable();
        }

        // Detener shooter

        // Detener transferencia
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }
        if (launcher != null){
            launcher.stopShooter();
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


        transferRunning = false;

        panelsTelemetry.debug("TELEOPTHETA detenido");
        panelsTelemetry.update(telemetry);

        // Limpiar subsistemas y comandos registrados
        CommandScheduler.getInstance().reset();
    }
}