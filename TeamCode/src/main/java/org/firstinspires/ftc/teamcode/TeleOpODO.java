package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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
    // Transferencia
    private DcMotorEx transferMotor;
    //Drivetrain
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
    public static double intakeVel = 1100;

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
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(launcher);
    }

    private void initializeTurret() {
        turret = new TurretSub(hardwareMap, "TurretMotor");
    }

    private void initializeShooter() {
        // flywheel constructor
        launcher = new LauncherSub(hardwareMap, "shooter","shooter2" ,"hood" );
    }

    private void initializeTransfer() {

        transferMotor = hardwareMap.get(DcMotorEx.class, "Transfer");

        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void initializeDrive() {

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

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initializeServo() {
        servoTope = new ServoEx(hardwareMap, "ServoTope");
    }


    @Override
    public void start() {

        turret.enable();


    }


    @Override
    public void loop() {



        CommandScheduler.getInstance().run();
        launcher.setGOAL_DISTANCE(turret.getGoalDistance());

        driveControl(gamepad1);
        servoControl(gamepad2);
        shooterControl(gamepad2);      // Toggle con botón A
        transferControl(gamepad2);
        turretControls(gamepad2);

        addShooterTelemetry();
        addTurretTelemetry();
        telemetry.update();

    }

    private void driveControl(Gamepad g1) {

        double y = -g1.left_stick_y;
        double x = g1.left_stick_x * 1.05;
        double rotation = g1.right_stick_x;

        double frontLeftPower = y + x + rotation;

        double frontRightPower = y - x - rotation;

        double backLeftPower = y - x + rotation;

        double backRightPower = y + x - rotation;

        double maximum = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

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
            transferMotor.setVelocity(intakeVel);
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
        telemetry.addData("Motor Velocity", launcher.getTicksPerSec());
        telemetry.addData("Motor Velocity Error", launcher.getTicksPerSecError());
    }

    private void addTurretTelemetry() {
        telemetry.addData("Goal Distance", turret.getGoalDistance());
        telemetry.addData("Turret Angle", turret.getCurrentAngle());
        telemetry.addData("Target Angle", turret.getTargetAngle());

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
        // Limpiar subsistemas y comandos registrados
        CommandScheduler.getInstance().reset();
    }
}