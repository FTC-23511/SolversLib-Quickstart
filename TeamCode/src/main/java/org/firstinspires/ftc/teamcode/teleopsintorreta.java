package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
@TeleOp(name = "TELEOPALPHA")
public class teleopsintorreta extends OpMode {



    DcMotorEx flywheelMotor, flywheelMotor2, Transfer;
    PIDFController shooterController;
    private static GoBildaPinpointDriver pinpon;

    ServoEx ServoTope;
    boolean shooterRunning = false;
    boolean transferRunning = false;


    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public static double targetVel = 700;
    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000525;
    public static double heading;





    @Override
    public void init() {
        flywheelMotor  = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Transfer       = hardwareMap.get(DcMotorEx.class, "Transfer");
        ServoTope = new ServoEx(hardwareMap, "ServoTope");





// por cada 4.8 vueltas del engrane del motor la torreta da una vuelta

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(kP, kI, kD, 0);
    }

    @Override
    public void loop() {

        heading = pinpon.getHeading(AngleUnit.DEGREES);



        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);


//
        if (gamepad2.left_bumper) {
            ServoTope.set(0.05);
        } else {
            ServoTope.set(0.5
            );
        }

        if (gamepad2.aWasPressed()) shooterRunning = !shooterRunning;



        if (shooterRunning) {
            double currentVelocity = (flywheelMotor.getVelocity() + flywheelMotor2.getVelocity()) / 2;
            shooterController.setPIDF(kP, kI, kD, 0);
            shooterController.setSetPoint(targetVel);
            telemetry.addData("velocidad: ", currentVelocity);

            double power = (kV * targetVel) + shooterController.calculate(currentVelocity);
            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);
        } else {
            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
        }


        if (gamepad2.yWasPressed()) transferRunning = !transferRunning;
        Transfer.setPower(transferRunning ? 1 : 0);
        telemetry.addData("heading ", heading);
        telemetry.update();
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }






}
