package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonCRServo;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.CoaxialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.drive.CoaxialSwerveModule;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversAxonServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversCRServo;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Photon
public class Robot {
    public SolversMotor liftLeft;
    public SolversMotor liftRight;
    public SolversMotor extension;
    public SolversMotor intakeMotor;

    public SolversMotor frontLeftMotor;
    public SolversMotor frontRightMotor;
    public SolversMotor backLeftMotor;
    public SolversMotor backRightMotor;

    public SolversServo rightClaw;
    public SolversServo leftClaw;
    public SolversServo rightArm;
    public SolversServo leftArm;
    public SolversServo wrist;
    public SolversServo tray;
    public SolversServo pitchingIntake;

    public SolversAxonServo frontLeftServo;
    public SolversAxonServo frontRightServo;
    public SolversAxonServo backLeftServo;
    public SolversAxonServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpendicularPod;

    public DistanceSensor intakeDistanceSensor;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BHI260IMU imu;
    private double rawIMUAngle = 0;
    public static double imuOffset = 0;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

    public Deposit deposit;
    public Intake intake;
    public CoaxialSwerveDrivetrain swerveDrivetrain;

    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        liftLeft = new SolversMotor("liftLeft", 0.01);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight = new SolversMotor("liftRight", 0.01);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension = new SolversMotor("extension", 0.01);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = new SolversMotor("intakeMotor", 0.01);

        frontLeftMotor = new SolversMotor("frontLeftMotor", 0.01);
        frontRightMotor = new SolversMotor("frontRightMotor", 0.01);
        backLeftMotor = new SolversMotor("backLeftMotor", 0.01);
        backRightMotor = new SolversMotor("backRightMotor", 0.01);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = new SolversServo("leftClaw", 0.0);
        rightClaw = new SolversServo("rightClaw", 0.0);
        leftArm = new SolversServo("leftArm", 0.0);
        rightArm = new SolversServo("rightArm", 0.0);
        wrist = new SolversServo("wrist", 0.0);
        tray = new SolversServo("tray", 0.0);
        pitchingIntake = new SolversServo("pitchingIntake", 0.0);

        frontLeftServo = new SolversAxonServo("frontLeftServo", 0.01);
        frontRightServo = new SolversAxonServo("frontRightServo", 0.01);
        backLeftServo = new SolversAxonServo("backLeftServo", 0.01);
        backRightServo = new SolversAxonServo("backRightServo", 0.01);

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArm.setDirection(Servo.Direction.REVERSE);

        frontLeftServo.setServoEncoder("frontLeftEncoder");
        frontLeftServo.setServoEncoder("frontRightEncoder");
        frontLeftServo.setServoEncoder("backLeftEncoder");
        frontLeftServo.setServoEncoder("backRightEncoder");


        liftEncoder = new MotorEx(hardwareMap, "liftRight").encoder;
        extensionEncoder = new MotorEx(hardwareMap, "extension").encoder;

        parallelPod = new MotorEx(hardwareMap, "backRightMotor").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpendicularPod = new MotorEx(hardwareMap, "backLeftMotor").encoder;
        perpendicularPod.setDirection(Motor.Direction.REVERSE);

        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");

        // IMU orientation
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }

        imuOffset = Globals.STARTING_HEADING;

        swerveDrivetrain = new CoaxialSwerveDrivetrain(
            new CoaxialSwerveModule[] {
                new CoaxialSwerveModule(frontLeftMotor, frontLeftServo, frontLeftEncoder, 0),
                new CoaxialSwerveModule(frontRightMotor, frontRightServo, frontRightEncoder, 0),
                new CoaxialSwerveModule(backLeftMotor, backLeftServo, backLeftEncoder, 0),
                new CoaxialSwerveModule(backRightMotor, backRightServo, backRightEncoder, 0)
            }
        );

        intake = new Intake();
        deposit = new Deposit();

        deposit.init();
        intake.init();

        // Add any OpMode specific initializations here
        if (Globals.opModeType == Globals.OpModeType.AUTO) {
            deposit.initAuto();
        } else {
            deposit.initTeleOp();
        }
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Globals.USING_IMU) {
            Thread imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        rawIMUAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    }
                }
            });
            imuThread.start();
        }
    }

    public void resetIMUOffset() {
        imuOffset = rawIMUAngle;
    }

    public double getAngle() {
        return rawIMUAngle - imuOffset;
    }
}
