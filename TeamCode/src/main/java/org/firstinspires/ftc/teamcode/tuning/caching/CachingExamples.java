package org.firstinspires.ftc.teamcode.tuning.caching;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/* IMPORTANT:
   - This file is purely for reference on how to implement Caching in motors and servos
   - This is a LinearOpMode instead of a OpMode so we can declare our motors publicly
 */

//@TeleOp
public class CachingExamples extends LinearOpMode {

    public CachingDcMotor motor1;
    public CachingDcMotorEX motor2;
    public CachingServo servo1;
    public CachingCRServo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = new CachingDcMotor(hardwareMap.get(DcMotor.class, "motorFrontLeft"));
        motor2 = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "motorBackLeft"));

        servo1 = new CachingServo(hardwareMap.get(CachingServo.class, "leftArm"));
        servo2 = new CachingCRServo(hardwareMap.get(CachingCRServo.class, "leftArm"));

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            assert true; // Functionally the same as a pass
        }
    }
}

