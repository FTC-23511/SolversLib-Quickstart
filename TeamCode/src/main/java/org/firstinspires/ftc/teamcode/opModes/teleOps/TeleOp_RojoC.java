package org.firstinspires.ftc.teamcode.opModes.teleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import Subsistemas.LauncherSub;
import Subsistemas.TurretSub;

@Configurable
@TeleOp(name = "TELEOP_rojo_C")
public class TeleOp_RojoC extends OpMode {

    private Follower follower;
    // Transferencia
    private DcMotorEx transferMotor;
    private double angle = 0;
    //Drivetrain
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    // Servo
    private ServoEx servoTope;
    // Subsistema de la torreta
    private ServoEx Light;
    private LauncherSub launcher;
    private TurretSub turret;

    private boolean reversetransfer = false;

    private boolean transferRunning;
    // ================= ESTADOS =================
    public static double servoMin = 0.0;
    public static double servoMax = 0.3;
    public static double intakeVel = 1500;

    public double hoodAngle = 0;
    public static double initX = 117.301;
    public static double initY = 126.059;

    public static Pose startingPose = new Pose(initX,initY,Math.toRadians(126.5));

    @Override
    public void init() {

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(
                LynxModule.BulkCachingMode.AUTO));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        follower.startTeleopDrive();

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().enable();


        initializeTurret();
        initializeShooter();
        initializeTransfer();
        initializeDrive();
        initializeServo();
        turret.setGoalX(144);
        turret.setManualAimOffsetDegrees(0);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(launcher);
    }

    private void initializeTurret() {
        turret = new TurretSub(hardwareMap, "TurretMotor");
    }

    private void initializeShooter() {
        // flywheel constructor
        launcher = new LauncherSub(hardwareMap, "shooter","shooter2" ,"hood" );
        launcher.setTargetTag(24);
    }

    private void initializeTransfer() {

        transferMotor = hardwareMap.get(DcMotorEx.class, "Transfer");

        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void initializeDrive() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

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
        servoTope = new ServoEx(hardwareMap, "ServoTope").setInverted(true);
        Light = new ServoEx(hardwareMap, "light");
    }


    @Override
    public void start() {

        turret.enable();


    }


    @Override
    public void loop() {
        follower.update();
        turret.setPose(follower.getPose());



        CommandScheduler.getInstance().run();

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
                -frontRightPower / maximum
        );
        backLeft.setPower(
                backLeftPower / maximum
        );
        backRight.setPower(
                -backRightPower / maximum
        );
    }
    private void servoControl(Gamepad g2) {
        double error;
        error = Math.abs(launcher.getTicksPerSecError());
        if (g2.right_bumper) {
            if (error <= 30)
                servoTope.set(servoMax);
            transferRunning = true;
        } else {
            servoTope.set(servoMin);

        }

        boolean shooterRunning = launcher.isShooterRunning();

        if (!shooterRunning && !transferRunning) {
            // Nada encendido -> Azul
            Light.set(0.61);

        } else if (!shooterRunning) {
            // Solo intake -> Rojo
            Light.set(0.28);

        } else if (!transferRunning) {
            // Solo shooter -> Verde (random)
            Light.set(0.39);

        } else {
            // Shooter + intake -> Amarillo (random)
            Light.set(0.50);
        }    }

    private void shooterControl(Gamepad g2) {
        if (g2.aWasPressed()){
            launcher.toggleShooter();
        }

        double v = 0.05;
        if(gamepad2.bWasPressed()) {
            hoodAngle = hoodAngle + v;
            launcher.setHOOD_ANGLE(hoodAngle);
        }
        if(gamepad2.xWasPressed()) {
            hoodAngle = hoodAngle - v;
            launcher.setHOOD_ANGLE(hoodAngle);
        }
    }

    private void transferControl(Gamepad g2) {

        if (g2.yWasPressed()) {
            transferRunning = !transferRunning;
            reversetransfer = false;
        }

        if (transferRunning) {
            transferMotor.setVelocity(intakeVel);
        } else {
            if (reversetransfer){
                transferMotor.setPower(-0.5);
            }
            else {transferMotor.setPower(0);}
        }

        if (g2.dpadUpWasPressed()){
            reversetransfer = !reversetransfer;
            transferRunning = false;
        }
    }


    private void turretControls(Gamepad gamepad2) {

        double offset = 1;

        if (gamepad1.xWasPressed()){
            angle =+ offset;
        }
        if (gamepad1.bWasPressed()){
            angle =- offset;
        }
        turret.setManualAimOffsetDegrees(angle);
    }

    private void addShooterTelemetry() {
        telemetry.addData("Motor Velocity", launcher.getTicksPerSec());
        telemetry.addData("Motor Velocity Error", launcher.getTicksPerSecError());

    }

    private void addTurretTelemetry() {
        telemetry.addData("Goal Distance", launcher.getDistance());
        telemetry.addData("Turret Angle", turret.getCurrentAngle());
        telemetry.addData("Target Angle", turret.getTargetAngle());
        telemetry.addData("Hood Angle" , hoodAngle);

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