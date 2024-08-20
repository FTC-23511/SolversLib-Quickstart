package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.caching.CachingCRServo;
import org.firstinspires.ftc.teamcode.hardware.caching.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.List;

public class RobotHardware {
    public CachingDcMotorEX liftLeft;
    public CachingDcMotorEX liftRight;
    public CachingDcMotorEX extension;
    public CachingDcMotorEX intakeMotor;

    public CachingDcMotorEX frontLeftMotor;
    public CachingDcMotorEX frontRightMotor;
    public CachingDcMotorEX backLeftMotor;
    public CachingDcMotorEX backRightMotor;

    public Servo rightClaw;
    public Servo leftClaw;
    public Servo rightArm;
    public Servo leftArm;
    public Servo wrist;
    public Servo tray;
    public Servo pitchingIntake;

    public CachingCRServo frontLeftServo;
    public CachingCRServo frontRightServo;
    public CachingCRServo backLeftServo;
    public CachingCRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;
    public Motor.Encoder parallelPod;
    public Motor.Encoder perpendicularPod;

    public DistanceSensor intakeDistanceSensor;

    public BHI260IMU imu;

    public List<LynxModule> allHubs;

    public Deposit deposit = new Deposit();
    public Intake intake = new Intake();

    private static RobotHardware instance = null;
    public boolean enabled;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        liftLeft = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "liftLeft"), 0.01);
        liftLeft.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "liftRight"), 0.01);
        liftRight.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "extension"), 0.01);
        extension.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "intakeMotor"), 0.01);

        frontLeftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "frontLeftMotor"), 0.01);
        frontRightMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "frontRightMotor"), 0.01);
        backLeftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "backLeftMotor"), 0.01);
        backRightMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "backRightMotor"), 0.01);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        tray = hardwareMap.get(Servo.class, "tray");
        pitchingIntake = hardwareMap.get(Servo.class, "pitchingIntake");

        frontLeftServo = new CachingCRServo(hardwareMap.get(CRServo.class, "frontLeftServo"), 0.01);
        frontRightServo = new CachingCRServo(hardwareMap.get(CRServo.class, "frontRightServo"), 0.01);
        backLeftServo = new CachingCRServo(hardwareMap.get(CRServo.class, "backLeftServo"), 0.01);
        backRightServo = new CachingCRServo(hardwareMap.get(CRServo.class, "backRightServo"), 0.01);

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        liftEncoder = new MotorEx(hardwareMap, "leftLift").encoder;
        liftEncoder.setDirection(Motor.Direction.REVERSE);
        extensionEncoder = new MotorEx(hardwareMap, "extension").encoder;

        parallelPod = new MotorEx(hardwareMap, "frontLeftMotor").encoder;
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        deposit.init();
        intake.init();
    }
}
